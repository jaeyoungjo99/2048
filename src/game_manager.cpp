#include "game_2048/game_manager.hpp"

// ----- Initialize -----
void GameManager::InitializeGame() {
    std::cout << "[GameManager] 2048 Initialize!" << std::endl;

    is_game_over_ = false;
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

    // Move the grid
    MoveGrid(direction);
    
    // Random Generate after move
    RandomGenerate();
}

// ----- Move Grid -----
void GameManager::MoveGrid(Direction direction) {
    // Rotate the grid
    RotateGrid(direction);

    int empty_cell_count = 0;
    // Move the grid
    for(int col = 0; col < GRID_NUM; col++) {
        bool found_valid_top = false;

        int column_processed[GRID_NUM] = {0};
        int last_top_num = 0;
        int row_proc_ind = 0;

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

                column_processed[row_proc_ind] = last_top_num;

                continue;
            }

            // cur num is not same as last top num
            if(found_valid_top == true && last_top_num != grid_[row][col]){
                last_top_num = grid_[row][col];

                row_proc_ind++;
                column_processed[row_proc_ind] = last_top_num;
                
                continue;
            }
                
            // cur num is same as last top num
            if(found_valid_top == true && last_top_num == grid_[row][col]){

                column_processed[row_proc_ind] = grid_[row][col] + 1;
                row_proc_ind++;

                found_valid_top = false;
            }
        }

        // Re fill the grid
        for(int row = 0; row < GRID_NUM; row++) {
            grid_[row][col] = column_processed[row];
            if(grid_[row][col] == 0) {
                empty_cell_count++;
            }
        }

    }

    // If there are no empty cells, check if the game is over
    if(empty_cell_count == 0) {
        is_game_over_ = true;
    }

    // Rotate back the grid
    RotateGrid(direction, true);
}

void GameManager::RotateGrid(Direction direction, bool reverse) {
    int temp_grid[GRID_NUM][GRID_NUM];

    // Copy the grid to temp_grid
    for(int i = 0; i < GRID_NUM; i++) {
        for(int j = 0; j < GRID_NUM; j++) {
            temp_grid[i][j] = grid_[i][j];
        }
    }

    // Rotate the grid
    if(direction == UP) {
        for(int i = 0; i < GRID_NUM; i++) {
            for(int j = 0; j < GRID_NUM; j++) {
                grid_[i][j] = temp_grid[GRID_NUM - i - 1][GRID_NUM - j - 1];
            }
        }
    }
    else if(direction == LEFT && reverse == false ||
            direction == RIGHT && reverse == true) {
        for(int i = 0; i < GRID_NUM; i++) {
            for(int j = 0; j < GRID_NUM; j++) {
                grid_[i][j] = temp_grid[GRID_NUM - j - 1][i];
            }
        }
    }
    else if(direction == RIGHT && reverse == false ||
            direction == LEFT && reverse == true) {
        for(int i = 0; i < GRID_NUM; i++) {
            for(int j = 0; j < GRID_NUM; j++) {
                grid_[i][j] = temp_grid[j][GRID_NUM - i - 1];
            }
        }
    }
    else{
        for(int i = 0; i < GRID_NUM; i++) {
            for(int j = 0; j < GRID_NUM; j++) {
                grid_[i][j] = temp_grid[i][j];
            }
        } 
    }
}

// ----- Check Game Over -----
bool GameManager::IsGameOver() {
    return is_game_over_;
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