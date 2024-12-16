#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Tỷ lệ chuyển đổi sang phần trăm
    constexpr float scale = 0.01f;

	if (start_x < 0 || start_x > 100 || start_y < 0 || start_y > 100 ||
        end_x < 0 || end_x > 100 || end_y < 0 || end_y > 100) {
        std::cerr << "Error: Coordinates must be in the range [0, 100]." << std::endl;
        return;
    }
    // Chuyển đổi tọa độ đầu vào sang phần trăm
    start_x *= scale;
    start_y *= scale;
    end_x *= scale;
    end_y *= scale;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	if (node == nullptr || end_node == nullptr) {
        std::cerr << "Error: The node is null." << std::endl;
		return -1;
    }

	return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Find neighbors
	if (current_node == nullptr) {
        std::cerr << "Error: Current node is null." << std::endl;
        return;
    }
    current_node->FindNeighbors();

    for (auto &node : current_node->neighbors) {
        if (!node->visited) {  
            node->parent = current_node;                              
            node->h_value = CalculateHValue(node);                    
            node->g_value = current_node->g_value + current_node->distance(*node); 
            node->visited = true;                                     
            open_list.emplace_back(node);                             
        }
    }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
	if (open_list.empty()) {
        std::cerr << "Error: open_list is empty, no more nodes to explore." << std::endl;
        return nullptr;
    }
    auto min_it = std::min_element(open_list.begin(), open_list.end(), [](const auto &a, const auto &b){
		if (a == nullptr || b == nullptr) {
            std::cerr << "Error: Encountered null node in open_list." << std::endl;
            return false;
        }
        return (a->g_value + a->h_value) < (b->g_value + b->h_value);
    });

    // Trích xuất nút có giá trị nhỏ nhất
    auto current = *min_it;
    open_list.erase(min_it);  // Xóa nút khỏi danh sách mở

    return current;  // Trả về nút được chọn
}



// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
	if (current_node == nullptr) {
        std::cerr << "Error: Current node is null, cannot construct final path." << std::endl;
        return {};  // Trả về vector rỗng nếu current_node là null
    }
    // Tạo vector chứa đường đi, dự đoán số lượng phần tử để tối ưu hóa cấp phát
    std::vector<RouteModel::Node> path_found;
    path_found.reserve(100);

    distance = 0.0f;
  
    // Lặp qua các node từ cuối đến đầu
    while (current_node->parent) {
        path_found.emplace_back(*current_node);  // Thêm node hiện tại vào đường đi
        distance += current_node->distance(*(current_node->parent));  // Tính khoảng cách
        current_node = current_node->parent;  // Di chuyển đến node cha
    }
    // Thêm node đầu tiên (không có parent)
    path_found.emplace_back(*current_node);
    // Đảo ngược đường đi để có thứ tự từ đầu đến cuối
    std::reverse(path_found.begin(), path_found.end());
    // Chuyển đổi khoảng cách sang mét
    distance *= m_Model.MetricScale();

    return path_found;
}



// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
	if (!start_node || !end_node) {
        std::cerr << "Error: Start or end node is invalid!" << std::endl;
        return;
    }
    // Đặt nút bắt đầu vào danh sách mở và đánh dấu là đã thăm
    RouteModel::Node *current_node = start_node;
    open_list.emplace_back(current_node);
    current_node->visited = true;

    // Vòng lặp chính của thuật toán A*
    while (!open_list.empty()) {
        // Lấy nút tiếp theo (có giá trị f thấp nhất)
        current_node = NextNode();
        // Kiểm tra nếu đã đến đích
        if (current_node == end_node) {
            // Kết thúc thuật toán
			m_Model.path = ConstructFinalPath(current_node);
			std::cout << "Path found! Total distance: " << distance << " meters." << std::endl;
			return;
        }
		if (!current_node) {
            std::cerr << "Error: Current node is invalid!" << std::endl;
            return;
        }
        // Thêm các neighbor của nút hiện tại vào danh sách mở
        AddNeighbors(current_node);
    }
    // Nếu vòng lặp kết thúc mà không tìm thấy đường đi
    std::cout << "No path found!" << std::endl;
}
