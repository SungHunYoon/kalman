NAME		= kalman

CXX			= c++
CXXFLAGS	= -Wall -Wextra -Werror -std=c++17
INCLUDES	= -Iinclude

SRC_DIR		= src
OBJ_DIR		= obj

SRCS		= $(shell find $(SRC_DIR) -name "*.cpp")
OBJS		= $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))
LIB_SRCS	= $(filter-out $(SRC_DIR)/main.cpp,$(SRCS))
TEST_SRCS	= $(shell find tests -name "*.cpp" 2>/dev/null)
HEADERS		= $(shell find include -name "*.hpp")
TEST_NAME	= kalman_tests
SANITIZE_NAME = kalman_tests_sanitize
SANITIZE_FLAGS = -fsanitize=address,undefined -fno-omit-frame-pointer
VISUALIZER_NAME = kalman_visualizer
VISUALIZER_SRCS = $(shell find visualizer -name "*.cpp") \
	src/visualizer/telemetry_receiver.cpp src/visualizer/view_model.cpp \
	src/visualizer/trajectory_buffer.cpp src/visualizer/render_math.cpp \
	src/telemetry/telemetry_packet.cpp src/performance/filter_stats.cpp
RAYLIB_FLAGS = $(shell pkg-config --cflags --libs raylib 2>/dev/null)

all: $(NAME)

$(NAME): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(NAME)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)

fclean: clean
	rm -f $(NAME) $(TEST_NAME) $(SANITIZE_NAME) $(VISUALIZER_NAME)

re: fclean all

test: $(TEST_NAME)
	./$(TEST_NAME)

sanitize: $(TEST_SRCS) $(LIB_SRCS)
	$(CXX) $(CXXFLAGS) $(SANITIZE_FLAGS) $(INCLUDES) $(TEST_SRCS) $(LIB_SRCS) -o $(SANITIZE_NAME)
	./$(SANITIZE_NAME)

$(TEST_NAME): $(TEST_SRCS) $(LIB_SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TEST_SRCS) $(LIB_SRCS) -o $(TEST_NAME)

visualizer:
	@if ! pkg-config --exists raylib; then \
		echo "raylib not found; install it with: brew install raylib" >&2; exit 1; \
	fi
	@$(MAKE) $(VISUALIZER_NAME)

$(VISUALIZER_NAME): $(VISUALIZER_SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(VISUALIZER_SRCS) $(RAYLIB_FLAGS) -o $(VISUALIZER_NAME)

.PHONY: all clean fclean re test sanitize visualizer
