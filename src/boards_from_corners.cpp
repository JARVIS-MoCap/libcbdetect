#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include <opencv2/opencv.hpp>

#include "board_energy.h"
#include "boards_from_corners.h"
#include "config.h"
#include "filter_board.h"
#include "grow_board.h"
#include "init_board.h"

namespace cbdetect {

void boards_from_corners(const cv::Mat& img, const Corner& corners, std::vector<Board>& boards, const Params& params) {
  // intialize boards
  boards.clear();
  Board board;
  std::vector<int> used(corners.p.size(), 0);

  int start = 0;
  if(!params.overlay) {
    // start from random index
    std::default_random_engine e;
    auto time = std::chrono::system_clock::now().time_since_epoch();
    e.seed(static_cast<unsigned long>(time.count()));
    start = e() % corners.p.size();
  }

  // for all seed corners do
  int n = 0;
  while(n++ < corners.p.size()) {
    // init 3x3 board from seed i
    int i = (n + start) % corners.p.size();
    if(used[i] == 1 || !init_board(corners, used, board, i)) {
      continue;
    }


    // check if this is a useful initial guess
    cv::Point3i maxE_pos = board_energy(corners, board, params);
    double energy        = board.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
    if(energy > -6.0) {
      for(int jj = 0; jj < 3; ++jj) {
        for(int ii = 0; ii < 3; ++ii) {
          used[board.idx[jj][ii]] = 0;
        }
      }
      continue;
    }

    // grow boards
    while(1) {
      int num_corners = board.num;

      for(int j = 0; j < (params.corner_type == MonkeySaddlePoint ? 6 : 4); ++j) {
        std::vector<cv::Point2i> proposal;
        GrowType grow_type = grow_board(corners, used, board, proposal, j, params);
        if(grow_type == GrowType_Failure) {
          continue;
        }

        filter_board(corners, used, board, proposal, energy, params);

        if(grow_type == GrowType_Inside) {
          --j;
        }
      }

      // exit loop
      if(board.num == num_corners) {
        break;
      }
    }

    if(!params.overlay) {
      boards.emplace_back(board);
      continue;
    }

    std::vector<std::pair<int, double>> overlap;
    for(int j = 0; j < boards.size(); ++j) {
      // check if new chessboard proposal overlaps with existing chessboards
      for(int k1 = 0; k1 < board.idx.size(); ++k1) {
        for(int k2 = 0; k2 < board.idx[0].size(); ++k2) {
          for(int l1 = 0; l1 < boards[j].idx.size(); ++l1) {
            for(int l2 = 0; l2 < boards[j].idx[0].size(); ++l2) {
              if(board.idx[k1][k2] != -1 && board.idx[k1][k2] != -2 && board.idx[k1][k2] == boards[j].idx[l1][l2]) {
                cv::Point3i maxE_pos_tmp = board_energy(corners, boards[j], params);
                overlap.emplace_back(std::make_pair(j, boards[j].energy[maxE_pos_tmp.y][maxE_pos_tmp.x][maxE_pos_tmp.z]));
                goto GOTO_BREAK;
              }
            }
          }
        }
      }
    }
  GOTO_BREAK:;

    if(overlap.empty()) {
      boards.emplace_back(board);
    } else {
      bool is_better = true;
      for(int j = 0; j < overlap.size(); ++j) {
        if(overlap[j].second <= energy) {
          is_better = false;
          break;
        }
      }
      if(is_better) {
        std::vector<Board> tmp;
        for(int j = 0, k = 0; j < boards.size(); ++j) {
          if(overlap[k].first == j) {
            continue;
            ++k;
          }
          tmp.emplace_back(boards[j]);
        }
        std::swap(tmp, boards);
        boards.emplace_back(board);
      }
    }
    std::fill(used.begin(), used.end(), 0);
    n += 2;
  }
}

} // namespace cbdetect
