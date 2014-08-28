dbclear all;

% compile matlab wrappers
disp('Building wrappers ...');
mex('matcherMex.cpp','../src/matcher.cpp','../src/filter.cpp','../src/triangle.cpp','../src/matrix.cpp','-I../src','CXXFLAGS=-msse3 -fPIC');
mex('visualOdometryStereoMex.cpp','../src/viso_stereo.cpp','../src/viso.cpp','../src/matcher.cpp','../src/filter.cpp','../src/triangle.cpp','../src/matrix.cpp','-I../src','CXXFLAGS=-msse3 -fPIC');
mex('visualOdometryMonoMex.cpp','../src/viso_mono.cpp','../src/viso.cpp','../src/matcher.cpp','../src/filter.cpp','../src/triangle.cpp','../src/matrix.cpp','-I../src','CXXFLAGS=-msse3 -fPIC');
mex('reconstructionMex.cpp','../src/reconstruction.cpp','../src/matrix.cpp','-I../src','CXXFLAGS=-msse3 -fPIC');
disp('...done!');

