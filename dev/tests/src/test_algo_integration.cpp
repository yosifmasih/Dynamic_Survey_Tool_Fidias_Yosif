//g++ -std=c++17 -I Software/lib/core/include \
  Software/dev/tests/src/test_algorithm.cpp \
  Software/lib/core/src/features.cpp \
  Software/lib/core/src/detectors.cpp \
  Software/lib/core/src/ml.cpp \
  Software/lib/core/src/ml_postproc.cpp \
  Software/lib/core/src/statemachine_ML.cpp \
  Software/lib/core/src/kalman.cpp \
  Software/lib/core/src/algorithm.cpp \
  -o /tmp/test_algorithm && /tmp/test_algorithm