/* Minimal host test harness. */
#ifndef TEST_H
#define TEST_H

#include <math.h>
#include <stdio.h>

extern int test_failures;
extern int test_checks;

#define CHECK(cond) do { test_checks++; if (!(cond)) { test_failures++; \
  printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { test_checks++; long long va_ = (long long)(a), vb_ = (long long)(b); \
  if (va_ != vb_) { test_failures++; \
  printf("FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__, __LINE__, #a, #b, va_, vb_); } } while (0)
#define CHECK_NEAR(a, b, tol) do { test_checks++; double va_ = (double)(a), vb_ = (double)(b); \
  if (!(fabs(va_ - vb_) <= (tol))) { test_failures++; \
  printf("FAIL %s:%d: %s ~= %s (%.9g != %.9g)\n", __FILE__, __LINE__, #a, #b, va_, vb_); } } while (0)

void test_can_proto(void);
void test_kinematics(void);
void test_pi_ctrl(void);
void test_drive_ctrl(void);
void test_encoder_math(void);
void test_cmd_timeout(void);
void test_ring_buf(void);

#endif /* TEST_H */
