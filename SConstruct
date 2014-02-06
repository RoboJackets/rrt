
env = Environment();
env.Replace(CXX='clang++')
env.Append(CXXFLAGS=['-std=c++11'])
env.Append(CPPPATH=['/usr/local/Cellar/eigen/3.2.0/include/eigen3/'])

# Fix to allow clang to show messages in color
import os
env['ENV']['TERM'] = os.environ['TERM']

Export('env');


SConscript('src/rrt-viewer/SConscript')


gtest = env.Command('gtest/make/gtest_main.a', 'gtest/make/makefile', 'make -C gtest/make/makefile')
Alias('gtest', gtest)
