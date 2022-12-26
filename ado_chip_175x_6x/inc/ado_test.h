/*
 * ado_test.h
 *
 *  Created on: 06.04.2020
 *      Author: Robert
 */

#ifndef ADO_TEST_H_
#define ADO_TEST_H_

#include <chip.h>

// type prototype. (needed because of self reference!)
typedef struct test_failure_s test_failure_t;
typedef struct test_s test_t;

// structure definition
typedef struct test_failure_s {
	test_failure_t* nextFailure;	// prototype can be used here!
	const char* 	fileName;
	const char* 	testName;
	int 			lineNr;
	char* 			message;
} test_failure_t;


typedef struct test_result_s {
	int				run;
	int				failed;
	test_failure_t* failures;
} test_result_t;

typedef struct test_s {
	const test_t *testBase;
	char *	name;
	uint8_t testCnt;
	void (*runIt)(test_result_t* result, const test_t* test);
} test_t;

#if TEST_CFG_MAXFAILURES
	#define TEST_MAX_FAILURES	TEST_CFG_MAXFAILURES
#else
	#define TEST_MAX_FAILURES	10
#endif

// Test API functions
void testRunAll(test_result_t* result, const test_t *test);

void testListSuites(const test_t *tests, uint8_t intend, void (*)(char *, uint8_t,  uint8_t));
const test_t* testFindSuite(const test_t* tests, char *name);
void testClearFailures();


void _testFailed(const char* file, const char* func, int ln, test_result_t* res, char* message);
#define testFailed(pRes, message) _testFailed(__FILE__, __func__, __LINE__, pRes, message)
#define testPassed(a)	 a->run++;

#define test_ok ((test_failure_t *)0)

#define TEST_CASE(fn) {0,0,1,(void (*)(test_result_t* result, const test_t* test))fn}
#define TEST_SUITE(name, tests) { tests, name, (uint8_t)(sizeof(tests) / sizeof(test_t)), testRunAll }

#define IS_TESTCASE(tstPtr) ((tstPtr->testCnt == 1) && (tstPtr->name == 0))
#define IS_TESTSUITE(tstPtr) (!IS_TESTCASE(tstPtr))

#endif /* ADO_TEST_H_ */
