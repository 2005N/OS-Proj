#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdbool.h>
#include <limits.h>

#define MAX_ELEVATORS 100
#define MAX_FLOORS 300
#define MAX_REQUESTS 2000
#define AUTH_STRING_MAX 20
#define MAX_SOLVERS 100
#define MAX_LOAD_LIMIT 20  // Elevator load limit
#define AUTH_GUESS_LIMIT 5000 // Limit attempts to avoid infinite loop

typedef struct {
    int requestId;
    int startFloor;
    int requestedFloor;
} PassengerRequest;

typedef struct {
    char authStrings[MAX_ELEVATORS][AUTH_STRING_MAX+1];
    char elevatorMovementInstructions[MAX_ELEVATORS];  // u, d, s
    PassengerRequest newPassengerRequests[30];
    int elevatorFloors[MAX_ELEVATORS];
    int droppedPassengers[1000];
    int pickedUpPassengers[1000][2];  // req id, elevator no
} MainSharedMemory;

typedef struct {
    long mtype;
    int elevatorNumber;
    char authStringGuess[AUTH_STRING_MAX];
} SolverRequest;

typedef struct {
    long mtype;
    int guessIsCorrect;
} SolverResponse;

typedef struct {
    long mtype;  // always 2
    int turnNumber;
    int newPassengerRequestCount;
    int errorOccured;
    int finished;
} TurnChangeResponse;

typedef struct {
    long mtype;  // must be 1
    int droppedPassengersCount;
    int pickedUpPassengersCount;
} TurnChangeRequest;

typedef struct {
    int front;
    int rear;
    PassengerRequest requests[1000];
    int size;
} RequestQueue;
void initQueue(RequestQueue *queue) {
    queue->front = -1;
    queue->rear = -1;
}

// Function to check if the queue is empty
bool isQueueEmpty(RequestQueue *queue) {
    return (queue->front == -1);
}
// Check if the queue is full
bool isQueueFull(RequestQueue *queue) {
    return ((queue->rear + 1) % MAX_REQUESTS == queue->front);
}

// Function to enqueue a request
bool enqueue(RequestQueue *queue, PassengerRequest request) {
    if (isQueueFull(queue)) {
        printf("Queue is full, cannot enqueue request %d\n", request.requestId);
        return false;
    }

    if (isQueueEmpty(queue)) {
        queue->front = 0;
    }
    queue->rear = (queue->rear + 1) % MAX_REQUESTS;
    queue->requests[queue->rear] = request;

    return true;
}

// Dequeue (remove) a request from the queue
bool dequeue(RequestQueue *queue, PassengerRequest *request) {
    if (isQueueEmpty(queue)) {
        printf("Queue is empty, cannot dequeue\n");
        return false;
    }

    *request = queue->requests[queue->front];
    if (queue->front == queue->rear) {
        queue->front = -1;
        queue->rear = -1;
    } else {
        queue->front = (queue->front + 1) % MAX_REQUESTS;
    }

    return true;
}

typedef struct {
    int requestId;
    int requestedFloor;
} OnboardPassenger;

OnboardPassenger elevatorPassengers[MAX_ELEVATORS][MAX_LOAD_LIMIT];
int passengerCounts[MAX_ELEVATORS] = {0};  // Keep track of the number of passengers per elevator


void increment_auth_string(char* authString, int length) {
    int i = length - 1;
    while (i >= 0) {
        if (authString[i] < 'f') {
            authString[i]++;
            break;
        } else {
            authString[i] = 'a';
            i--;
        }
    }
}
int generate_auth_string(char* authString, int solver_msg_queue, int elevatorId, int passengerCount) {
    SolverRequest solverReq = {2, elevatorId, ""};
    msgsnd(solver_msg_queue, &solverReq, sizeof(solverReq) - sizeof(long), 0);

    int attempts = 0;
    while (attempts < AUTH_GUESS_LIMIT) {
        if (strlen(authString) == 0) {
            for (int i = 0; i < passengerCount; i++) {
                authString[i] = 'a';
            }
            authString[passengerCount] = '\0';
        } else {
            increment_auth_string(authString, passengerCount);
        }
        strncpy(solverReq.authStringGuess, authString, passengerCount);
        solverReq.mtype = 3;
        msgsnd(solver_msg_queue, &solverReq, sizeof(solverReq) - sizeof(long), 0);

        SolverResponse solverRes;
        msgrcv(solver_msg_queue, &solverRes, sizeof(solverRes) - sizeof(long), 4, 0);

        if (solverRes.guessIsCorrect) {
            return 1; // Success
        }
        attempts++;
    }
    return 0; // Failed to find correct auth string in limited attempts
}

struct CompareContext {
        int *elevatorFloors;
        int numElevators;
};
int compareRequests(const void *a, const void *b, void *context) {
    PassengerRequest *reqA = (PassengerRequest *)a;
    PassengerRequest *reqB = (PassengerRequest *)b;
    
    struct CompareContext *ctx = (struct CompareContext *)context;

    int *floors = ctx->elevatorFloors;
    int numElevators = ctx->numElevators;

    int minDistA = abs(floors[0] - reqA->startFloor);
    int minDistB = abs(floors[0] - reqB->startFloor);

    for (int i = 1; i < numElevators; i++) {
        int distA = abs(floors[i] - reqA->startFloor);
        int distB = abs(floors[i] - reqB->startFloor);

        if (distA < minDistA) minDistA = distA;
        if (distB < minDistB) minDistB = distB;
    }

    // Primary sort by proximity to the closest elevator
    if (minDistA != minDistB) {
        return minDistA - minDistB;
    }

    // Secondary sort by direction preference 
    int directionA = (reqA->requestedFloor - reqA->startFloor);
    int directionB = (reqB->requestedFloor - reqB->startFloor);

    if (directionA != directionB) {
        return directionA - directionB;
    }

    // If still equal, sort by request ID 
    return reqA->requestId - reqB->requestId;
}
static struct CompareContext globalContext;
int compareRequestsGlobal(const void *a, const void *b) {
    return compareRequests(a, b, &globalContext);
}
void sortRequestsByProximityAndDirection(RequestQueue *queue, int *elevatorFloors, int numElevators) {
    if (queue->size > 1) {
        globalContext.elevatorFloors = elevatorFloors;
        globalContext.numElevators = numElevators;

        qsort(queue->requests, queue->size, sizeof(PassengerRequest), compareRequestsGlobal);
    }
}
int findBestRequest(RequestQueue *queue, int currentFloor, PassengerRequest *bestRequest) {
    int bestIndex = -1;
    int minDistance = INT_MAX;

    if (isQueueEmpty(queue)) {
        return 0;
    }

    int size = (queue->rear >= queue->front) ? (queue->rear - queue->front + 1) : (1000 - queue->front + queue->rear + 1);
    for (int i = 0; i < size; i++) {
        int index = (queue->front + i) % 1000;
        PassengerRequest *request = &queue->requests[index];
        int distance = abs(currentFloor - request->startFloor);

        if (distance < minDistance) {
            minDistance = distance;
            *bestRequest = *request;
            bestIndex = index;
        }
    }

    // Remove the selected request from the queue if a suitable one was found
    if (bestIndex != -1) {
        if (bestIndex == queue->front) {
            dequeue(queue, bestRequest); 
        } else {
            // Shift the remaining elements to fill the gap
            for (int i = bestIndex; i != queue->rear; i = (i + 1) % 1000) {
                queue->requests[i] = queue->requests[(i + 1) % 1000];
            }
            queue->rear = (queue->rear - 1 + 1000) % 1000; // Update rear
            if (queue->rear == -1) {
                queue->front = -1;
            }
        }
        return 1;
    }

    return 0;
}

int main(){
    int N, M, K, T;
    key_t shm_key, helper_msg_key;
    key_t solver_msg_keys[MAX_SOLVERS];

    FILE *input_file = fopen("input.txt", "r");
    if (input_file == NULL) {
        perror("Failed to open input file");
        exit(EXIT_FAILURE);
    }

    fscanf(input_file, "%d", &N); // Number of Elevators
    fscanf(input_file, "%d", &K); // Number of Floors
    fscanf(input_file, "%d", &M); // Number of Solvers
    fscanf(input_file, "%d", &T); // Turn number of last request
    fscanf(input_file, "%d", &shm_key); // Key for shared memory
    fscanf(input_file, "%d", &helper_msg_key); // Key for main message queue
    
    for (int i = 0; i < M; i++) {
        fscanf(input_file, "%d", &solver_msg_keys[i]);
    }
    fclose(input_file);
    int passengerCounts[N];
    for (int i = 0; i < N; i++) {
        passengerCounts[i] = 0; 
    }

    MainSharedMemory *mainShmPtr;
    int shm_id = shmget(shm_key, sizeof(MainSharedMemory), IPC_CREAT | 0666);
    if (shm_id == -1) {
        perror("Failed to create shared memory");
        exit(EXIT_FAILURE);
    }
    mainShmPtr = (MainSharedMemory*)shmat(shm_id, NULL, 0);
    if (mainShmPtr == (void*) -1) {
        perror("Failed to attach shared memory");
        exit(EXIT_FAILURE);
    }

    int helper_msg_queue = msgget(helper_msg_key, IPC_CREAT | 0666);
    if (helper_msg_queue == -1) {
        perror("msgget for helper failed");
        exit(EXIT_FAILURE);
    }

    int solver_msg_queues[MAX_SOLVERS];
    for (int i = 0; i < M; i++) {
        solver_msg_queues[i] = msgget(solver_msg_keys[i], IPC_CREAT | 0666);
        if (solver_msg_queues[i] == -1) {
            perror("msgget for solver failed");
            exit(EXIT_FAILURE);
        }
    }

    RequestQueue newRequestQueue[T];
    // Initialize all queues
    for (int i = 0; i < T; i++) {
        initQueue(&newRequestQueue[i]);
    }
    
    //printf("M: %d  N: %d\n",M,N);

TurnChangeResponse turnResponse;
    msgrcv(helper_msg_queue, &turnResponse, sizeof(turnResponse) - sizeof(long), 2, 0);
    
    int turn = turnResponse.turnNumber;
    if (turnResponse.errorOccured) exit(EXIT_FAILURE);

    int newRequests = turnResponse.newPassengerRequestCount;

    for (int i = 0; i < newRequests; i++) {
        enqueue(&newRequestQueue[turn], mainShmPtr->newPassengerRequests[i]);
        //printf("Request added: %d\n", mainShmPtr->newPassengerRequests[i].requestId);
    }

while (1) {

    int droppedPassengersCount = 0;
    int pickedUpPassengersCount = 0;

    // Initialize all elevators to 'stay'
    for (int i = 0; i < N; i++) {
        mainShmPtr->elevatorMovementInstructions[i] = 's';
    }

    // Process requests and move elevators
    for (int elevatorId = 0; elevatorId < N; elevatorId++) {
        int currentFloor = mainShmPtr->elevatorFloors[elevatorId];
        PassengerRequest frontRequest;

        if (findBestRequest(&newRequestQueue[turn], currentFloor, &frontRequest)){
            int reqId = frontRequest.requestId;
                int startFloor = frontRequest.startFloor;
                int targetFloor = frontRequest.requestedFloor;
                //printf("debug %d\n",newRequestQueue[turn].front);
    
                // Move to the passenger's start floor
                while (currentFloor != startFloor) {
                    if (currentFloor < startFloor && currentFloor < K - 1) {
                        mainShmPtr->elevatorMovementInstructions[elevatorId] = 'u';
                    } else if (currentFloor > startFloor && currentFloor > 0) {
                        mainShmPtr->elevatorMovementInstructions[elevatorId] = 'd';
                    } else {
                        break;
                    }
                    
                    if(passengerCounts[elevatorId] > 0){
                        generate_auth_string(mainShmPtr->authStrings[elevatorId], solver_msg_queues[0], elevatorId, passengerCounts[elevatorId]);
                    }
    
                    //printf("Elevator %d moving from floor %d to %d.\n", elevatorId, currentFloor, startFloor);
    
                    // Process movement
                    TurnChangeRequest turnRequest = {1, 0, 0};
                    msgsnd(helper_msg_queue, &turnRequest, sizeof(turnRequest) - sizeof(long), 0);
    
                    msgrcv(helper_msg_queue, &turnResponse, sizeof(turnResponse) - sizeof(long), 2, 0);
                    if (turnResponse.finished) break;
                    if (turnResponse.errorOccured) {
                        printf("Error during movement for elevator %d.\n", elevatorId);
                        exit(EXIT_FAILURE);
                    }
                    
                    newRequests = turnResponse.newPassengerRequestCount;
                    // Enqueue new requests
                    for (int i = 0; i < newRequests; i++) {
                        enqueue(&newRequestQueue[turn], mainShmPtr->newPassengerRequests[i]);
                        //printf("Request added: %d\n", mainShmPtr->newPassengerRequests[i].requestId);
                    }
                    
    
                    currentFloor = mainShmPtr->elevatorFloors[elevatorId];
                }
    
                // Pick up the passenger
                if (currentFloor == startFloor && passengerCounts[elevatorId] < MAX_LOAD_LIMIT) {
                    mainShmPtr->pickedUpPassengers[0][0] = reqId;
                    mainShmPtr->pickedUpPassengers[0][1] = elevatorId;
                    passengerCounts[elevatorId]++;pickedUpPassengersCount++;
    
                    elevatorPassengers[elevatorId][passengerCounts[elevatorId] - 1].requestId = reqId;
                    elevatorPassengers[elevatorId][passengerCounts[elevatorId] - 1].requestedFloor = targetFloor;
    
                    //printf("Elevator %d picked up passenger %d at floor %d.\n", elevatorId, reqId, startFloor);
                    for (int i = 0; i < N; i++) mainShmPtr->elevatorMovementInstructions[i] = 's';
    
                    if (currentFloor < targetFloor && currentFloor < K - 1) {
                        mainShmPtr->elevatorMovementInstructions[elevatorId] = 'u';
                    } else if (currentFloor > targetFloor && currentFloor > 0) {
                        mainShmPtr->elevatorMovementInstructions[elevatorId] = 'd';
                    }
                    if(passengerCounts[elevatorId] > 0){
                        generate_auth_string(mainShmPtr->authStrings[elevatorId], solver_msg_queues[0], elevatorId, passengerCounts[elevatorId]);
                    }
                    TurnChangeRequest turnRequest = {1, 0, 1};
                    msgsnd(helper_msg_queue, &turnRequest, sizeof(turnRequest) - sizeof(long), 0);
    
                    msgrcv(helper_msg_queue, &turnResponse, sizeof(turnResponse) - sizeof(long), 2, 0);
                    if (turnResponse.finished) break;
                    if (turnResponse.errorOccured) {
                        printf("Error during movement for elevator %d.\n", elevatorId);
                        exit(EXIT_FAILURE);
                    }
                    
                    newRequests = turnResponse.newPassengerRequestCount;
                    // Enqueue new requests
                    for (int i = 0; i < newRequests; i++) {
                        enqueue(&newRequestQueue[turn], mainShmPtr->newPassengerRequests[i]);
                        //printf("Request added: %d\n", mainShmPtr->newPassengerRequests[i].requestId);
                    }
    
                    currentFloor = mainShmPtr->elevatorFloors[elevatorId];
                }
    
             
                while (targetFloor != -1 && currentFloor != targetFloor) {
                    if (currentFloor < targetFloor && currentFloor < K - 1) {
                        mainShmPtr->elevatorMovementInstructions[elevatorId] = 'u';
                    } else if (currentFloor > targetFloor && currentFloor > 0) {
                        mainShmPtr->elevatorMovementInstructions[elevatorId] = 'd';
                    } else {
                        break;
                    }
                    
                    if(passengerCounts[elevatorId] > 0){
                        generate_auth_string(mainShmPtr->authStrings[elevatorId], solver_msg_queues[0], elevatorId, passengerCounts[elevatorId]);
                    }
    
                    //printf("Elevator %d moving towards target floor %d from floor %d auth-%s.\n", elevatorId, targetFloor, currentFloor,mainShmPtr->authStrings[elevatorId]);
    
                    // Process movement
                    TurnChangeRequest turnRequest = {1, 0, 0};
                    msgsnd(helper_msg_queue, &turnRequest, sizeof(turnRequest) - sizeof(long), 0);
    
                    msgrcv(helper_msg_queue, &turnResponse, sizeof(turnResponse) - sizeof(long), 2, 0);
                    if (turnResponse.finished) break;
                    if (turnResponse.errorOccured) {
                        printf("Error during movement for elevator %d.\n", elevatorId);
                        exit(EXIT_FAILURE);
                    }
                    
                    newRequests = turnResponse.newPassengerRequestCount;
                    // Enqueue new requests
                    for (int i = 0; i < newRequests; i++) {
                        enqueue(&newRequestQueue[turn], mainShmPtr->newPassengerRequests[i]);
                        //printf("Request added: %d\n", mainShmPtr->newPassengerRequests[i].requestId);
                    }
    
                    currentFloor = mainShmPtr->elevatorFloors[elevatorId];
                }
    
                // Drop off passengers
                //printf("Passenger count for elevator %d: %d\n", elevatorId, passengerCounts[elevatorId]);
                for (int i = 0; i < passengerCounts[elevatorId]; i++) {
                    //printf("Elevator %d has passenger %d targeting floor %d.\n", elevatorId, elevatorPassengers[elevatorId][i].requestId, elevatorPassengers[elevatorId][i].requestedFloor);
            
                    if (targetFloor == currentFloor) {
                        mainShmPtr->droppedPassengers[0] = elevatorPassengers[elevatorId][i].requestId;
                        //printf("Elevator %d dropping passenger %d at floor %d.\n", elevatorId, elevatorPassengers[elevatorId][i].requestId, currentFloor);
                        droppedPassengersCount++;
                        // Shift passengers
                        for (int j = i; j < passengerCounts[elevatorId] - 1; j++) {
                            elevatorPassengers[elevatorId][j] = elevatorPassengers[elevatorId][j + 1];
                        }
                        passengerCounts[elevatorId]--;
                        i--;  // Adjust index after shift
                        for (int i = 0; i < N; i++) mainShmPtr->elevatorMovementInstructions[i] = 's';
    
                        TurnChangeRequest turnRequest = {1, 1, 0};
                        msgsnd(helper_msg_queue, &turnRequest, sizeof(turnRequest) - sizeof(long), 0);
        
                        msgrcv(helper_msg_queue, &turnResponse, sizeof(turnResponse) - sizeof(long), 2, 0);
                        if (turnResponse.finished) break;
                        if (turnResponse.errorOccured) {
                            printf("Error during movement for elevator %d.\n", elevatorId);
                            exit(EXIT_FAILURE);
                        }
                        
                        newRequests = turnResponse.newPassengerRequestCount;
                    // Enqueue new requests
                    for (int i = 0; i < newRequests; i++) {
                        enqueue(&newRequestQueue[turn], mainShmPtr->newPassengerRequests[i]);
                        //printf("Request added: %d\n", mainShmPtr->newPassengerRequests[i].requestId);
                    }
        
                        currentFloor = mainShmPtr->elevatorFloors[elevatorId];
                    }     
                }
                
        }
    }
    

}

    return 0;
} 