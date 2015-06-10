// These calls are used to initialize and operate on semaphores.
// We use system V semaphore calls for this simple typically included cpp compatible code

#ifndef _IPC_SEM_FUNCTIONS    // {
#define  _IPC_SEM_FUNCTIONS

// Semaphore includes from System V
#include<sys/sem.h>
#include<sys/ipc.h>
#include<sys/types.h>

#define IPC_SEM_ERR_FAILURE		-1
#define IPC_SEM_ERR_NO_SEM_YET		-2

//  ipc_sem_init returns a semid after initializing it's initial count to the specified count
//  Return semid OR -1 if there was a failure in setup of the semaphore
//         For -1 failure errno will contain the specific fault
//
//  NOTe: If you just want a lock it is easier to use ipc_sem_init_lock()
//        ipc_sem_init() is only required if a counting sem is required 
//        to allocate some fixed number of resources like a pool of resources.   
//
int ipc_sem_init(key_t key, unsigned short initial_sem_value) {

  int semid = semget(key, 1, IPC_CREAT);
  if (semid < 0) {
    return IPC_SEM_ERR_FAILURE;          // errno will contain the more specific error code
  }

  // Set starting counting value for the semaphore
  if (semctl(semid,0, SETVAL, initial_sem_value) == -1) {
    return IPC_SEM_ERR_FAILURE;          // errno will contain the more specific error code
  }

  return semid;         // Success!  This is the semaphore ID for later calls
}


//  ipc_sem_init_lock returns a semid after initializing it's initial count to 1
//  Return semid OR -1 if there was a failure in setup of the semaphore
//         For -1 failure errno will contain the specific fault
//
int ipc_sem_init_lock(key_t key) {
  unsigned short sem_value = 1;         // This call only supports a simple lock of count 1
  
  int semid = ipc_sem_init(key, sem_value);
  if (semid < 0) {
    return IPC_SEM_ERR_FAILURE;          // errno will contain the more specific error code
  }

  return semid;         // Success!  This is the semaphore ID for later calls
}

//  ipc_sem_get_by_key returns a semid for an already created system v semaphore 
//          This routine uses the given key to find the semaphore
//
//  Return semid OR IPC_SEM_ERR_FAILURE if there was a failure in setup of the semaphore
//         For IPC_SEM_ERR_FAILURE failure errno will contain the specific fault
//         Return IPC_SEM_ERR_NO_SEM_YET if the error was ENOENT which means it may not exist yet
//
int ipc_sem_get_by_key(key_t key) {

  int semid = semget(key, 0, 0);
  if (semid < 0) {
    if (errno == ENOENT) {
      return IPC_SEM_ERR_NO_SEM_YET;	// it may not exist yet so the caller can wait and retry
    }
    return IPC_SEM_ERR_FAILURE;          // errno will contain the more specific error code
  }

  return semid;         // Success!  This is the semaphore ID for later calls
}

//  ipc_sem_get_by_key_with_wait  will wait for specified time before failure
//
//  The wait time is in seconds but we try every 1/10 of a second 
//
//  This call allows waiting for another resource to form the lock
int ipc_sem_get_by_key_with_wait(key_t key, int wait_in_seconds) {
  int semid;
  int numSecWait = wait_in_seconds * 10;

  do {
    semid = ipc_sem_get_by_key(key);

    if (semid >= 0) {
      return semid;         // Success!  This is the semaphore ID for later calls
    }

    if (semid == IPC_SEM_ERR_NO_SEM_YET) {	// it may not exist yet so the caller can wait and retry
      numSecWait -= 1;
      usleep(100000);
      continue;
    } else {
      break;					// for a general error we abort
    }

  } while (numSecWait > 0);

  return semid;			// This is a failure case 

}


// ipc_sem_lock() will block until the semaphore is available then we MUST release it later
// Return 0 if we got the semaphore or return -1 if there was some failure.
//        For -1 failure errno will contain the specific fault
//
// If 0 is the return value, the caller MUST release the sem no matter WHAT fault!
int ipc_sem_lock(int semid) {

  struct  sembuf sem_wait;
  sem_wait.sem_num = 0;
  sem_wait.sem_op = -1;
  sem_wait.sem_flg = SEM_UNDO;

  if (semop(semid, &sem_wait, 1)) {
    return -1;
  }

  return 0;   // We have this semaphore locked.  

  // IMPORTANT!  Caller MUST release the sem no matter WHAT fault!
}

// ipc_sem_unlock() will unlock the simple lock semaphore specified by the sem id
// Return 0 if the unlock went fine
//        For -1 failure errno will contain the specific fault
//        I think if you unlock twice this call may exit in an error case.
//
// Note that this call could be used for counting semaphores too
int ipc_sem_unlock(int semid) {
  struct  sembuf sem_signal;

  sem_signal.sem_num = 0;
  sem_signal.sem_op = 1;
  sem_signal.sem_flg = SEM_UNDO;

  if (semop(semid, &sem_signal, 1) == -1) {
    return -1;          // errno will hold the reason for this failure case
  }

  return 0;             // the lock is released
}

#endif   //  }  _IPC_SEM_FUNCTIONS
