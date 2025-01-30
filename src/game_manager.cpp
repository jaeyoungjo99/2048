#include "game_2048/game_manager.hpp"

// ----- Initialize -----
void GameManager::InitializeGame() {
    std::cout << "[GameManager] 2048 Initialize!" << std::endl;

    // Initialize the grid with 0
    for(int i = 0; i < GRID_NUM; i++) {
        for(int j = 0; j < GRID_NUM; j++) {
            grid_[i][j] = 0;
        }
    }

    RandomGenerate();
}

// ----- Random Generate -----
void GameManager::RandomGenerate() {
    // Find all empty positions (0s) in the grid
    std::vector<std::pair<int, int>> empty_positions;
    for (int i = 0; i < GRID_NUM; ++i) {
        for (int j = 0; j < GRID_NUM; ++j) {
            if (grid_[i][j] == 0) {
                empty_positions.emplace_back(i, j); // Store the position of empty cells
            }
        }
    }

    // If there are no empty positions, return
    if (empty_positions.empty()) {
        return;
    }

    // Randomly select one of the empty positions
    int random_index = rand() % empty_positions.size();
    auto position = empty_positions[random_index];

    // Generate a random value (1 with 90% probability, 2 with 10% probability)
    int value = (rand() % 10 < 9) ? 1 : 2; // 90% chance for 1, 10% chance for 2

    // Place the value in the selected position
    grid_[position.first][position.second] = value;
}

void GameManager::UpdateGame(Direction direction) {
    std::cout << "[GameManager] 2048 Update!" << std::endl;

    // Move the grid    
    switch(direction) {
        case UP:
            MoveUp();
            break;
        case DOWN:
            MoveDown();
            break;
        case LEFT:
            MoveLeft();
            break;
        case RIGHT:
            MoveRight();
            break;
    }


    
    // Random Generate after move
    RandomGenerate();
}

bool GameManager::IsGameOver() {
    return false; // TODO: Implement this function
}


// ----- Move the grid -----
// TODO: Rotate and Movd Down!
void GameManager::MoveUp() {
    std::cout << "[GameManager] Move Up!" << std::endl;
    for(int col = 0; col < GRID_NUM; col++) {
        int num_processed = 0;
        bool found_valid_top = false;

        int number_processed[GRID_NUM] = {0};
        int last_top_num = 0;
        int p_row = 0;

        // Check merging
        for(int row = GRID_NUM - 1; row >= 0; row--) {
            if(grid_[row][col] == 0) {
                // Skip if the grid is empty
                continue;
            }

            // Find the top number  
            if(found_valid_top == false && grid_[row][col] != 0){
                found_valid_top = true;
                last_top_num = grid_[row][col];

                number_processed[p_row] = last_top_num;

                continue;
            }

            // cur num is not same as last top num
            if(found_valid_top == true && last_top_num != grid_[row][col]){
                last_top_num = grid_[row][col];

                p_row++;
                number_processed[p_row] = last_top_num;
                
                continue;
            }
                
            // cur num is same as last top num
            if(found_valid_top == true && last_top_num == grid_[row][col]){

                number_processed[p_row] = grid_[row][col] + 1;
                p_row++;

                found_valid_top = false;
            }
        }

        // Re fill the grid
        for(int row = 0; row < GRID_NUM; row++) {
            grid_[row][col] = number_processed[GRID_NUM-row-1];
        }

    }
}

void GameManager::MoveDown() {
    std::cout << "[GameManager] Move Down!" << std::endl;
    for(int col = 0; col < GRID_NUM; col++) {
        int num_processed = 0;
        bool found_valid_top = false;

        int number_processed[GRID_NUM] = {0};
        int last_top_num = 0;
        int p_row = 0;

        // Check merging
        for(int row = 0; row < GRID_NUM; row++) {
            if(grid_[row][col] == 0) {
                // Skip if the grid is empty
                continue;
            }

            // Find the top number  
            if(found_valid_top == false && grid_[row][col] != 0){
                found_valid_top = true;
                last_top_num = grid_[row][col];

                number_processed[p_row] = last_top_num;

                continue;
            }

            // cur num is not same as last top num
            if(found_valid_top == true && last_top_num != grid_[row][col]){
                last_top_num = grid_[row][col];

                p_row++;
                number_processed[p_row] = last_top_num;
                
                continue;
            }
                
            // cur num is same as last top num
            if(found_valid_top == true && last_top_num == grid_[row][col]){

                number_processed[p_row] = grid_[row][col] + 1;
                p_row++;

                found_valid_top = false;
            }
        }

        // Re fill the grid
        for(int row = 0; row < GRID_NUM; row++) {
            grid_[row][col] = number_processed[row];
        }

    }
}

void GameManager::MoveLeft() {
    std::cout << "[GameManager] Move Left!" << std::endl;
    for(int row = 0; row < GRID_NUM; row++) {
        int num_processed = 0;
        bool found_valid_left = false;

        int number_processed[GRID_NUM] = {0};
        int last_left_num = 0;
        int p_col = 0;

        // Check merging
        for(int col = 0; col < GRID_NUM; col++) {
            if(grid_[row][col] == 0) {
                // Skip if the grid is empty
                continue;
            }

            // Find the left number  
            if(found_valid_left == false && grid_[row][col] != 0){
                found_valid_left = true;
                last_left_num = grid_[row][col];

                number_processed[p_col] = last_left_num;

                continue;
            }

            // cur num is not same as last left num
            if(found_valid_left == true && last_left_num != grid_[row][col]){
                last_left_num = grid_[row][col];

                p_col++;
                number_processed[p_col] = last_left_num;
                
                continue;
            }
                
            // cur num is same as last left num
            if(found_valid_left == true && last_left_num == grid_[row][col]){
                number_processed[p_col] = grid_[row][col] + 1;
                p_col++;

                found_valid_left = false;
            }
        }

        // Re fill the grid
        for(int col = 0; col < GRID_NUM; col++) {
            grid_[row][col] = number_processed[col];
        }
    }
}

void GameManager::MoveRight() {
    std::cout << "[GameManager] Move Right!" << std::endl;
    for(int row = 0; row < GRID_NUM; row++) {
        int num_processed = 0;
        bool found_valid_right = false;

        int number_processed[GRID_NUM] = {0};
        int last_right_num = 0;
        int p_col = 0;

        // Check merging
        for(int col = GRID_NUM - 1; col >= 0; col--) {
            if(grid_[row][col] == 0) {
                // Skip if the grid is empty
                continue;
            }

            // Find the right number  
            if(found_valid_right == false && grid_[row][col] != 0){
                found_valid_right = true;
                last_right_num = grid_[row][col];

                number_processed[p_col] = last_right_num;

                continue;
            }

            // cur num is not same as last right num
            if(found_valid_right == true && last_right_num != grid_[row][col]){
                last_right_num = grid_[row][col];

                p_col++;
                number_processed[p_col] = last_right_num;
                
                continue;
            }
                
            // cur num is same as last right num
            if(found_valid_right == true && last_right_num == grid_[row][col]){
                number_processed[p_col] = grid_[row][col] + 1;
                p_col++;

                found_valid_right = false;
            }
        }

        // Re fill the grid
        for(int col = 0; col < GRID_NUM; col++) {
            grid_[row][GRID_NUM - col - 1] = number_processed[col]; // Fill from the right
        }
    }
}

// ----- Getter -----
void GameManager::GetGrid(int grid[GRID_NUM][GRID_NUM]) {
    // std::cout << "[GameManager] GetGrid!" << std::endl;

    for(int i = 0; i < GRID_NUM; i++) {
        for(int j = 0; j < GRID_NUM; j++) {
            grid[i][j] = grid_[i][j];
        }
    }
}   

void GameManager::GetScore(int& score) {
    score = 0;
    int max_number = 0;
    for(int i = 0; i < GRID_NUM; i++) {
        for(int j = 0; j < GRID_NUM; j++) {
            if(grid_[i][j] > max_number) {
                max_number = grid_[i][j];
            }
        }
    }

    score = std::pow(2, max_number);
}