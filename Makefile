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

all: $(NAME)

$(NAME): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(NAME)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(HEADERS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)

fclean: clean
	rm -f $(NAME) $(TEST_NAME) $(SANITIZE_NAME)

re: fclean all

test: $(TEST_NAME)
	./$(TEST_NAME)

sanitize: $(TEST_SRCS) $(LIB_SRCS)
	$(CXX) $(CXXFLAGS) $(SANITIZE_FLAGS) $(INCLUDES) $(TEST_SRCS) $(LIB_SRCS) -o $(SANITIZE_NAME)
	./$(SANITIZE_NAME)

$(TEST_NAME): $(TEST_SRCS) $(LIB_SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TEST_SRCS) $(LIB_SRCS) -o $(TEST_NAME)

.PHONY: all clean fclean re test sanitize
