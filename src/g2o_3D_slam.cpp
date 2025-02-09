// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <g2o/core/block_solver.h>
#include <g2o/core/eigen_types.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

// #include "s_graphs/backend/graph_slam.hpp"

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

// template <int N, typename T = double>
// using MatrixN = Eigen::Matrix<T, N, N, Eigen::ColMajor>;
// using MatrixX = MatrixN<Eigen::Dynamic>;

int n_vertices = 0;
int n_edges = 0;
std::unique_ptr<g2o::SparseOptimizer> graph;  // g2o graph

std::pair<int, g2o::VertexSE3 *> add_se3_node(const Eigen::Isometry3d & pose)
{
  g2o::VertexSE3 * vertex(new g2o::VertexSE3());
  auto id = n_vertices++;
  vertex->setId(static_cast<int>(id));
  vertex->setEstimate(pose);
  graph->addVertex(vertex);
  return {id, vertex};
}

g2o::EdgeSE3 * add_se3_edge(
  g2o::VertexSE3 * v1,
  g2o::VertexSE3 * v2,
  const Eigen::Isometry3d & relative_pose,
  const Eigen::MatrixXd & information_matrix)
{
  g2o::EdgeSE3 * edge(new g2o::EdgeSE3());
  edge->setId(static_cast<int>(n_edges++));
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);
  return edge;
}

void optimize()
{
  g2o::SparseOptimizer * graph = dynamic_cast<g2o::SparseOptimizer *>(::graph.get());
  const int num_iterations = 100;

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size()
            << std::endl;
  std::cout << "optimizing... " << std::flush;

  std::cout << "init" << std::endl;
  graph->initializeOptimization();
  graph->setVerbose(false);

  double chi2 = graph->chi2();
  if (std::isnan(graph->chi2())) {
    std::cout << "GRAPH RETURNED A NAN WAITING AFTER OPTIMIZATION" << std::endl;
  }
  std::cout << "optimize!!" << std::endl;
  int iterations = graph->optimize(num_iterations);
  std::cout << "done" << std::endl;
  std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  if (std::isnan(graph->chi2())) {
    throw std::invalid_argument("GRAPH RETURNED A NAN...STOPPING THE EXPERIMENT");
  }
}

void generate_graph()
{
  std::vector<g2o::VertexSE3 *> nodes;
  // robot1 will go through positions 1,0 -> 2,0 -> 3,0 (z = 0 allways)
  // first node is fixed to 0,0,0
  auto [id0, ground] = add_se3_node(Eigen::Isometry3d::Identity());
  ground->setFixed(true);
  // this position will help the optimizer to find the correct solution ( its like the
  // prior )
  nodes.emplace_back(ground);

  /* auto pos2 = Eigen::Isometry3d::Identity();
  pos2.translation() = Eigen::Vector3d(1, 0, 0);
  auto [node2, id2] = add_se3_node(pos2);
  auto pos3 = Eigen::Isometry3d::Identity();
  pos3.translation() = Eigen::Vector3d(2, 0, 0);
  auto [node3, id3] = add_se3_node(pos3);
  auto pos4 = Eigen::Isometry3d::Identity();
  pos4.translation() = Eigen::Vector3d(3, 0, 0);
  auto [node4, id4] = add_se3_node(pos4); */

  Eigen::Isometry3d relative_pose;
  relative_pose.setIdentity();
  auto prev_node = ground;

  for (int i : {1, 2, 3, 4, 5}) {
    Eigen::Vector3d traslation(0, 0, i);
    auto [id, node] = add_se3_node(Eigen::Isometry3d::Identity());
    nodes.emplace_back(node);
    relative_pose.translation() = traslation;
    add_se3_edge(prev_node, node, relative_pose, Eigen::MatrixXd::Identity(6, 6));

    std::cout << "nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size()
              << std::endl;
    node->estimate().translation().transpose();

    optimize();
    for (auto p : graph->vertices()) {
      // for (pair[ int id, VertexSE3 node ] : graph->vertices()) {
      int id = p.first;
      auto node = dynamic_cast<g2o::VertexSE3 *>(p.second);
      if (node) {
        auto T = node->estimate().translation().transpose();
        auto R = node->estimate().rotation().transpose();
        std::cout << "NODE " << id << " : " << T << " : " << R << std::endl;
      }
      // else { Node is not VertexSE3 }
    }
    // std::cout << "robot1" << std::endl;
    // std::cout << "ground1: " << ground1->estimate().translation().transpose() << std::endl;
    // std::cout << "node2: " << node2->estimate().translation().transpose() << std::endl;
    // std::cout << "node3: " << node3->estimate().translation().transpose() << std::endl;
    // std::cout << "node4: " << node4->estimate().translation().transpose() << std::endl;
  }

  // std::cout << graph->vertices() << std::endl;
  // optimize();

  // for (auto p : graph->vertices()) {
  //   // for (pair[ int id, VertexSE3 node ] : graph->vertices()) {
  //   int id = p.first;
  //   std::cout << id << std::endl;
  //   auto T = nodes[id]->estimate().translation().transpose();
  //   std::cout << "T " << id << " : " << T << std::endl;
  //   //
  //   // std::cout << node << std::endl;
  //   // std::cout << node->id() << std::endl;

  //   //   // auto id = node[0];
  //   //   // auto v  = node[1];
  //   // auto T = node.estimate().translation().transpose();
  //   // std::cout << "T : " << T << std::endl;
  // }

  // {
  //   // auto [node2, id2] = add_se3_node(Eigen::Isometry3d::Identity());
  //   // auto [node3, id3] = add_se3_node(Eigen::Isometry3d::Identity());
  //   // auto [node4, id4] = add_se3_node(Eigen::Isometry3d::Identity());

  //   // add edges
  //   // relative_pose.translation() = Eigen::Vector3d(1, 0, 0);
  //   // add_se3_edge(node2, node3, relative_pose, Eigen::MatrixXd::Identity(6, 6));
  //   // relative_pose.translation() = Eigen::Vector3d(1, 0, 0);
  //   // add_se3_edge(node3, node4, relative_pose, Eigen::MatrixXd::Identity(6, 6));

  //   // now we will create a second robot that will go through positions -3,0 -> -2,0 ->
  //   // -1,0 but they origin will be undetermined,
  //   //
  //   auto [ground2, id_g2] = add_se3_node(Eigen::Isometry3d::Identity());
  //   auto [node5, id5]     = add_se3_node(Eigen::Isometry3d::Identity());
  //   auto [node6, id6]     = add_se3_node(Eigen::Isometry3d::Identity());
  //   auto [node7, id7]     = add_se3_node(Eigen::Isometry3d::Identity());

  //   // add edges
  //   // now we will put the edges related to the ground node 2
  //   //
  //   relative_pose.setIdentity();
  //   relative_pose.translation() = Eigen::Vector3d(-3, 0, 0);
  //   add_se3_edge(ground2, node5, relative_pose, Eigen::MatrixXd::Identity(6, 6));
  //   relative_pose.translation() = Eigen::Vector3d(-2, 0, 0);
  //   add_se3_edge(ground2, node6, relative_pose, Eigen::MatrixXd::Identity(6, 6));
  //   relative_pose.translation() = Eigen::Vector3d(-1, 0, 0);
  //   add_se3_edge(ground2, node7, relative_pose, Eigen::MatrixXd::Identity(6, 6));

  //   // relative_pose.setIdentity();
  //   // relative_pose.translation() = Eigen::Vector3d(-3, 0, 0);
  //   // add_se3_edge(ground2, node5, relative_pose, Eigen::MatrixXd::Identity(6, 6));
  //   // relative_pose.translation() = Eigen::Vector3d(1, 0, 0);
  //   // add_se3_edge(node5, node6, relative_pose, Eigen::MatrixXd::Identity(6, 6));
  //   // relative_pose.translation() = Eigen::Vector3d(1, 0, 0);
  //   // add_se3_edge(node6, node7, relative_pose, Eigen::MatrixXd::Identity(6, 6));

  //   // now we want to create a loop closure between the two robots, we will say that node2
  //   // and node 6 are the same we will add a loop closure between node2 and node5

  //   relative_pose.setIdentity();
  //   add_se3_edge(node2, node7, relative_pose, Eigen::MatrixXd::Identity(6, 6));

  //   optimize();

  //   // log the results
  //   std::cout << "robot1" << std::endl;
  //   std::cout << "ground1: " << ground1->estimate().translation().transpose() << std::endl;
  //   std::cout << "node2: " << node2->estimate().translation().transpose() << std::endl;
  //   std::cout << "node3: " << node3->estimate().translation().transpose() << std::endl;
  //   std::cout << "node4: " << node4->estimate().translation().transpose() << std::endl;

  //   std::cout << "robot2" << std::endl;
  //   std::cout << "ground2: " << ground2->estimate().translation().transpose() << std::endl;
  //   std::cout << "node5: " << node5->estimate().translation().transpose() << std::endl;
  //   std::cout << "node6: " << node6->estimate().translation().transpose() << std::endl;
  //   std::cout << "node7: " << node7->estimate().translation().transpose() << std::endl;

  //   // obtain the covariance matrix for robot2 (each edge from ground2 to node5,
  //   // node6,node7

  //   // g2o::SparseBlockMatrix<MatrixX> spinv;
  //   g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;
  //   auto cov = graph->computeMarginals(spinv, ground2);
  //   // g2o::SparseBlockMatrix<Eigen::MatrixXd>::SparseMatrixBlock* b =
  //   //     spinv.blockCols()[spinv.blockCols().size() - 1].begin()->second;
  //   // auto inverse_b = b->inverse();
  //   std::cout << "covariance\n" << spinv << std::endl;
  //   std::cout << spinv.block(0, 0) << std::endl;
  //   // std::cout << "covariance matrix for robot2" << std::endl;
  //   // std::cout << spinv << std::endl;
}

int main()
{
  std::string solver_type = "lm_var_cholmod";
  ::graph = std::make_unique<g2o::SparseOptimizer>();
  g2o::SparseOptimizer * graph = dynamic_cast<g2o::SparseOptimizer *>(::graph.get());

  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory * solver_factory =
    g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm * solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if (!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return 0;
  }

  generate_graph();

  return 0;
}
