#构建目录
BUILD_DIR = ./build

#可执行文件生成目录
DEST_DIR = ./build

#可执行文件名
DEST_EXE_NAME = example

#以下命令直接在工程下目录进行，不必切换到build目录下

#一键make, 调用方式: make all/make
all:
	cd $(BUILD_DIR); make -j4

#一键执行可执行文件, 调用方式: make run
run:
	cd $(DEST_DIR); ./$(DEST_EXE_NAME)

#一键清除build目录，调用方式: make clean
clean:
	rm -rf $(BUILD_DIR)

#一键创建build目录并cmake，调用方式: make build
build:
	mkdir $(BUILD_DIR); cd $(BUILD_DIR); cmake ..

#一键清除build目录并且cmake，调用方式: make recmake
recmake:
	rm -rf $(BUILD_DIR); mkdir $(BUILD_DIR); cd $(BUILD_DIR); cmake ..

#一键清除build目录并且make，调用方式: make remake
remake:
	rm -rf $(BUILD_DIR); mkdir $(BUILD_DIR); cd $(BUILD_DIR); cmake ..; make
