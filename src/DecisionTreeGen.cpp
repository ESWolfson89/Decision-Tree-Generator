// Decision Tree Generator for Random Datasets
// Copyright (c) Eric Wolfson 2016 - 2017
// See LICENSE.txt for license/copyright details (MIT)
// Generates a decision tree using the OpenGL GLFW API with:
// 1) The ID3 algorithm for creating the tree
// 2) Walker's algorithm for displaying the nodes in the tree
// TO DO: 1) Split into several source files
//        2) Make better sliders
//        3) Make data values clearer, i.e. 1850 can be split up 
//           (in terms of attribute-property) 18-50, 1-850, 185-0, etc...
//           This can be done by color-coding.

#define GLFW_INCLUDE_GLU

#include <glfw3.h>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <queue>
#include <vector>
#include <set>
#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>

#define TEXT_TILE_WIDTH 8.0f
#define TEXT_TILE_HEIGHT 8.0f

#define TEXT_BITMAP_ROWS 16.0f
#define TEXT_BITMAP_COLS 16.0f

#define FONT_BITMAP_XSIZE 128.0f
#define FONT_BITMAP_YSIZE 128.0f

#define MAX_NODES 7
#define MAX_LEVELS 4

#define AREA_WID 1200
#define AREA_HGT 700

#define MAX_SAME_SAMPLE_THRESHOLD 200

#define SCALE_FACTOR 1.0f
#define NEGATIVE_INF -999999.0f

#define SCALE_OFFSETX ((float)AREA_WID / 2.0f)
#define SCALE_OFFSETY 12.0f

#define DEFAULT_NUM_ATTS 5
#define DEFAULT_NUM_SAMPS 5
#define DEFAULT_NUM_PROPCOMBS 2

#define MAX_NUM_ATTS 50
#define MAX_NUM_SAMPS 100
#define MAX_NUM_PROPCOMBS 30

struct node_loc_data
{
    double x_coord;
    double y_coord;
    double prelim;
    double modifier;
};

struct node_info
{
    int id;
    int leaf_id;
    int level;
    int num_children;
    int radius;
    bool is_root;
};

struct node_stats
{
    std::string label;
    std::string feedin_branch_value;
};

struct node
{
    node_stats stats;
    node_info info;
    node_loc_data loc_data;
    node *parent;
    node *left_sibling;
    node *right_sibling;
    node *left_neighbor;
    node *children;
};

std::string double2String(double);

std::string int2String(int);

/*
 * Class for holding data of each node and each node's position
 */
 
class treeEngine
{
public:
    treeEngine();
    ~treeEngine();
    void buildTree();
    void freeTree(node *);
    void createRootNode(std::string);
    void createChildNodes(node *, std::vector<std::string>&, int);
    void printTreeData(node *);
    void setAllLeftNeighbors(node *);
    void doAllFirstWalksForPostOrder();
    void firstWalk(node *);
    void secondWalk(node *, double);
    void apportion(node *);
    void offsetLocations(node *);
    void setupTreeNodePositions();
    void setNumLevels();
    void resetAllLocData(node *);
    void resetAllLeftNeighbors(node *);
    node *getRootNode();
    node *getLeftMostDescendant(node *, int);
    node *nextNodeInPostOrderTraversal(node *);
    double getMeanNodeSize(node *, node *);
private:
    node *root;
    int id_counter;
    int leaf_counter;
    int num_levels;
    double level_coord_increment;
    double min_sibling_coord_separation;
    double min_subtree_coord_separation;
    double x_top_adjustment;
    double y_top_adjustment;
    double bottom_y_coord;
    double lowest_x_coord;
    double highest_x_coord;
};

treeEngine::treeEngine()
{
    root = NULL;
    id_counter = 0;
    leaf_counter = 0; 
    level_coord_increment = 48.0;
    min_sibling_coord_separation = 36.0;
    min_subtree_coord_separation = 24.0;
    num_levels = 0;
    highest_x_coord = SCALE_OFFSETX;
    lowest_x_coord = SCALE_OFFSETX;
    bottom_y_coord = SCALE_OFFSETY;
}

treeEngine::~treeEngine()
{
    freeTree(root);
}

node * treeEngine::getRootNode()
{
    return root;
}

void treeEngine::resetAllLocData(node *n)
{
    if (n->children == NULL)
        return;

    if (n->info.is_root)
    {
        n->loc_data.x_coord = 0.0;
        n->loc_data.y_coord = 0.0;
        n->loc_data.prelim = 0.0;
        n->loc_data.modifier = 0.0;
    }

    for (int i = 0; i < n->info.num_children; ++i)
    {
        n->children[i].loc_data.x_coord = 0.0;
        n->children[i].loc_data.y_coord = 0.0;
        n->children[i].loc_data.prelim = 0.0;
        n->children[i].loc_data.modifier = 0.0;
        resetAllLocData(&(n->children[i]));
    }
}

void treeEngine::setNumLevels()
{
    num_levels = MAX_LEVELS;
}

void treeEngine::offsetLocations(node *n)
{
    n->loc_data.x_coord += SCALE_OFFSETX;
    n->loc_data.y_coord += SCALE_OFFSETY;

    if (n->children == NULL)
        return;

    for (int i = 0; i < n->info.num_children; ++i)
    {
        offsetLocations(&(n->children[i]));
    }
}

void treeEngine::printTreeData(node *n)
{
    if (n->children == NULL)
        return;

    for (int i = 0; i < n->info.num_children; ++i)
    {
        std::cout << "node <" << n->children[i].info.id << "> is connected to node <" <<
            n->info.id << "> and has level (" << n->children[i].info.level << ")\n";
        std::cout << "parent: " << n->info.id;
        if (n->children[i].left_sibling != NULL)
            std::cout << " left sibling: " << n->children[i].left_sibling->info.id;
        else
            std::cout << " left sibling: NULL";

        if (n->children[i].right_sibling != NULL)
            std::cout << " right sibling: " << n->children[i].right_sibling->info.id << "\n\n";
        else
            std::cout << " right sibling: NULL\n\n";

        if (n->children[i].left_neighbor != NULL)
            std::cout << " left neighbor: " << n->children[i].left_neighbor->info.id << "\n\n";
        else
            std::cout << " left neighbor: NULL\n\n";

        std::cout << "x: " << n->children[i].loc_data.x_coord << " y: " << n->children[i].loc_data.y_coord << "\n\n";

        printTreeData(&(n->children[i]));
    }
}

void treeEngine::setAllLeftNeighbors(node *n)
{
    std::queue<node*> node_queue;

    if (n)
        if (n != NULL)
            node_queue.push(n);

    while (!node_queue.empty())
    {
        // review this line:
        node * temp = node_queue.front();

        node_queue.pop();

        if (!node_queue.empty())
            if (temp->info.level == node_queue.front()->info.level && node_queue.front()->left_neighbor == NULL)
            {
                node_queue.front()->left_neighbor = temp;
            }

        for (int i = 0; i < temp->info.num_children; ++i)
        {
            node_queue.push(&(temp->children[i]));
        }
    }
}

void treeEngine::doAllFirstWalksForPostOrder()
{
    node *n = root;
    while (n->children != NULL)
        n = &(n->children[0]);
    while (n != NULL)
    {
        firstWalk(n);
        n = nextNodeInPostOrderTraversal(n);
    }
}

node *treeEngine::nextNodeInPostOrderTraversal(node *n)
{
    if (n->right_sibling != NULL)
    {
        node *temp = n->right_sibling;
        while (temp->children != NULL && temp->info.num_children >= 1)
            temp = &(temp->children[0]);
        return temp;
    }
    else
    {
        if (n->parent)
            return n->parent;
        else
            return NULL;
    }
}

// first walk of Walker's Algorithm
void treeEngine::firstWalk(node *n)
{
    n->loc_data.modifier = 0.0;

    if (n->children == NULL)
    {
        if (n->left_sibling != NULL)
            n->loc_data.prelim = n->left_sibling->loc_data.prelim + min_sibling_coord_separation + getMeanNodeSize(n->left_sibling, n);
        else
            n->loc_data.prelim = 0.0;
    }
    else
    {
        node *left_child = &(n->children[0]);
        node *right_child = &(n->children[n->info.num_children - 1]);
        double mid_point = (left_child->loc_data.prelim + right_child->loc_data.prelim) / 2.0;
        if (n->left_sibling != NULL)
        {
            n->loc_data.prelim = n->left_sibling->loc_data.prelim + min_sibling_coord_separation + getMeanNodeSize(n->left_sibling, n);
            n->loc_data.modifier = n->loc_data.prelim - mid_point;
            apportion(n);
        }
        else
            n->loc_data.prelim = mid_point;
    }
}

void treeEngine::apportion(node *n)
{
    node *current_left = n;
    node *left_neighbor = n->left_neighbor;
    double compare_depth = 0.0, left_modifier_sum, right_modifier_sum, move_distance, left_siblings, portion;
    while (current_left != NULL && left_neighbor != NULL)
    {
        left_modifier_sum = right_modifier_sum = 0;
        node *leftmost_ancestor = current_left;
        node *neighbor_ancestor = left_neighbor;
        for (int i = 0; i < compare_depth; ++i)
        {
            leftmost_ancestor = leftmost_ancestor->parent;
            neighbor_ancestor = neighbor_ancestor->parent;
            right_modifier_sum += leftmost_ancestor->loc_data.modifier;
            left_modifier_sum += neighbor_ancestor->loc_data.modifier;
        }
        move_distance = left_neighbor->loc_data.prelim + left_modifier_sum + getMeanNodeSize(current_left, left_neighbor) + min_subtree_coord_separation -
                        current_left->loc_data.prelim - right_modifier_sum;
        if (move_distance > 0)
        {
            node *temp = n;
            left_siblings = 0;
            while (temp != NULL && temp != neighbor_ancestor)
            {
                left_siblings++;
                temp = temp->left_sibling;
            }
            if (temp != NULL)
            {
                portion = move_distance / left_siblings;
                temp = n;
                while (temp != NULL && temp != neighbor_ancestor)
                {
                    temp->loc_data.prelim += move_distance;
                    temp->loc_data.modifier += move_distance;
                    move_distance -= portion;
                    temp = temp->left_sibling;
                }
            }
            else
                return;
        }
        compare_depth++;
        current_left = getLeftMostDescendant(n, (int)compare_depth);
        if (current_left != NULL)
            left_neighbor = current_left->left_neighbor;
    }
}

node *treeEngine::getLeftMostDescendant(node *n, int level)
{
    if (level <= 0)
        return n;
    else if (n->children == NULL)
        return NULL;
    else
    {
        node * ancestor = &(n->children[0]);
        node * left_most = getLeftMostDescendant(ancestor, level - 1);
        while (left_most == NULL && ancestor->right_sibling != NULL)
        {
            ancestor = ancestor->right_sibling;
            left_most = getLeftMostDescendant(ancestor, level - 1);
        }
        return left_most;
    }
}

double treeEngine::getMeanNodeSize(node *left, node *right)
{
    int node_size = 0;

    if (left != NULL)
        node_size += left->info.radius;
    if (right != NULL)
        node_size += right->info.radius;

    return (double)node_size;
}

void treeEngine::secondWalk(node *n, double modifier)
{
    n->loc_data.x_coord = x_top_adjustment + n->loc_data.prelim + modifier;
    n->loc_data.y_coord = y_top_adjustment + (n->info.level * level_coord_increment);
    if (n->children != NULL)
        secondWalk(&(n->children[0]), modifier + n->loc_data.modifier);
    if (n->right_sibling != NULL)
        secondWalk(n->right_sibling, modifier);
}

void treeEngine::setupTreeNodePositions()
{
    doAllFirstWalksForPostOrder();
    x_top_adjustment = root->loc_data.x_coord - root->loc_data.prelim;
    y_top_adjustment = root->loc_data.y_coord;
    secondWalk(root, 0.0);
}

void treeEngine::resetAllLeftNeighbors(node *n)
{
    if (n->children == NULL)
        return;

    for (int i = 0; i < n->info.num_children; ++i)
    {
        if (i == 0)
            n->children[i].left_neighbor = NULL;
        else
            n->children[i].left_neighbor = &(n->children[i - 1]);

        resetAllLeftNeighbors(&(n->children[i]));
    }
}

// root insert function
void treeEngine::createRootNode(std::string vertex_label)
{
    root = new node;
    root->stats.label = vertex_label;
    root->stats.feedin_branch_value = "";
    root->children = NULL;
    root->parent = NULL;
    root->left_sibling = NULL;
    root->left_neighbor = NULL;
    root->right_sibling = NULL;
    root->info.id = 0;
    root->info.is_root = true;
    root->info.num_children = 0;
    root->info.level = 0;
    root->info.radius = 4;
    root->loc_data.x_coord = 0.0;
    root->loc_data.y_coord = 0.0;
    root->loc_data.prelim = 0.0;
    root->loc_data.modifier = 0.0;
    root->info.leaf_id = -1;

    return;
}

// child insert function
void treeEngine::createChildNodes(node *n, std::vector<std::string> &edge_labels, int sub_nodes)
{
    if (sub_nodes == 0)
        return;

    n->children = new node[sub_nodes];
    n->info.num_children = sub_nodes;

    for (int i = 0; i < sub_nodes; ++i)
    {
        id_counter++;
        n->children[i].stats.label = "NL";
        n->children[i].stats.feedin_branch_value = edge_labels[i];
        n->children[i].children = NULL;
        n->children[i].parent = n;
        n->children[i].info.id = id_counter;
        n->children[i].info.is_root = false;
        n->children[i].info.num_children = 0;
        n->children[i].info.level = n->info.level + 1;
        n->children[i].info.radius = 4;
        n->children[i].loc_data.x_coord = 0.0;
        n->children[i].loc_data.y_coord = 0.0;
        n->children[i].loc_data.prelim = 0.0;
        n->children[i].loc_data.modifier = 0.0;
        n->children[i].info.leaf_id = -1;
    }

    for (int i = 0; i < sub_nodes; ++i)
    {
        if (i == 0)
            n->children[i].left_sibling = n->children[i].left_neighbor = NULL;
        else
            n->children[i].left_sibling = n->children[i].left_neighbor = &(n->children[i - 1]);

        if (i == sub_nodes - 1)
            n->children[i].right_sibling = NULL;
        else
            n->children[i].right_sibling = &(n->children[i + 1]);
    }

    return;
}

// clean up all nodes
void treeEngine::freeTree(node *n)
{
    if (!n)
        return;

    if (n == NULL)
        return;

    if (n->children == NULL && n->info.is_root)
    {
        delete n;
        return;
    }

    if (n->children != NULL)
    {
        for (int i = 0; i < n->info.num_children; ++i)
        {
            freeTree(&(n->children[i]));
        }
    }
    else if (!n->info.is_root)
        return;

    delete[] n->children;

    if (n->info.is_root)
    {
        delete n;
        std::cout << "\nGraph node memory freed.\n\n";
        std::cout << "\nPress enter to exit.\n";
        std::cin.get();
    }
}

void treeEngine::buildTree()
{
    root = new node;
    root->info.num_children = (rand () % MAX_NODES) + 2;
    root->info.id = 0;
    root->info.radius = 1;
    root->info.level = 0;
    root->parent = root->right_sibling = root->left_sibling = root->left_neighbor = NULL;
    root->loc_data.prelim = root->loc_data.modifier = 0.0;
    root->loc_data.x_coord = 0.0;
    root->loc_data.y_coord = 0.0;
    setAllLeftNeighbors(root);
}

/*
 * Class for computing the ID3 algorithm for a random dataset
 */

class dataEngine
{
public:
    dataEngine();
    ~dataEngine();
    void initTree();
    bool randomlyPopulateDataset(int,int,int);
    bool sampleAlreadyExists(int);
    double calculateRelativeEntropy(std::vector<int>&);
    double calculateGain(std::vector<int>&, int);
    int findMaximumGainPropertyIndex(std::vector<int>&, std::vector<int>&);
    std::string getDataValue(int, int);
    std::string getAttributeName(int);
    std::string mostCommonResultInSample(std::vector<int>&);
    int getNumSamples();
    int getNumProperties();
    node * getRootNode();
    void setupNodePositions();
    void ID3Algorithm(std::vector<int>&, std::vector<int>&, node *);
private:
    std::vector<std::vector<std::string>> dataset;
    std::vector<std::string> attribute_names;
    std::vector<int> num_property_combinations;
    int num_samples;
    int num_properties;
    treeEngine tree_obj;
};

dataEngine::dataEngine()
{

}

dataEngine::~dataEngine()
{
    std::vector<std::vector<std::string>>().swap(dataset);
    std::vector<int>().swap(num_property_combinations);
    std::vector<std::string>().swap(attribute_names);
}

void dataEngine::setupNodePositions()
{
    tree_obj.setAllLeftNeighbors(tree_obj.getRootNode());
    tree_obj.setupTreeNodePositions();
    tree_obj.offsetLocations(tree_obj.getRootNode());
}

node * dataEngine::getRootNode()
{
    return tree_obj.getRootNode();
}

void dataEngine::ID3Algorithm(std::vector<int>& sample_indices, std::vector<int>& property_indices, node * insert_node)
{
    bool all_equal = true;
    int decision = 0;

    if ((int)property_indices.size() == 0)
        return;

    std::string repeated_value = dataset[sample_indices[0]][num_properties - 1];
    std::vector<std::string> edge_labels;
    std::vector<int> new_sample_indices;
    std::vector<int> new_property_indices;

    for (int i = 1; i < (int)sample_indices.size(); ++i)
    {
        if (repeated_value != dataset[sample_indices[i]][num_properties - 1])
        {
            all_equal = false;
        }
    }

    if (all_equal)
    {
        if (tree_obj.getRootNode() == NULL)
        {
            tree_obj.createRootNode(repeated_value);
            return;
        }
        else
        {
            insert_node->stats.label = repeated_value;
            return;
        }
    }

    decision = findMaximumGainPropertyIndex(sample_indices, property_indices);

    if (insert_node != NULL)
        insert_node->stats.label = attribute_names[decision];

    for (int i = 0; i < (int)property_indices.size(); ++i)
    {
        if (property_indices[i] != decision)
        {
            new_property_indices.push_back(property_indices[i]);
        }
    }

    for (int i = 0; i < num_property_combinations[decision]; ++i)
    {
        edge_labels.push_back(int2String(decision) + int2String(i));
    }

    if (tree_obj.getRootNode() == NULL)
    {
        tree_obj.createRootNode(attribute_names[decision]);
        insert_node = tree_obj.getRootNode();
    }

    tree_obj.createChildNodes(insert_node, edge_labels, (int)edge_labels.size());

    for (int i = 0; i < insert_node->info.num_children; ++i)
    {
        for (int j = 0; j < (int)sample_indices.size(); ++j)
        {
            if (dataset[sample_indices[j]][decision] == edge_labels[i])
            {
                new_sample_indices.push_back(sample_indices[j]);
            }
        }
        if ((int)new_sample_indices.size() == 0)
        {
            insert_node->children[i].stats.label = mostCommonResultInSample(sample_indices);
            continue;
        }
        ID3Algorithm(new_sample_indices, new_property_indices, &(insert_node->children[i]));
        new_sample_indices.clear();
    }
}

void dataEngine::initTree()
{
    tree_obj = treeEngine::treeEngine();
}

std::string dataEngine::mostCommonResultInSample(std::vector<int> & sample_indices)
{
    std::vector<int> counts;
    int max_count = 0;
    int index = 0;

    counts.resize(num_property_combinations[num_properties - 1]);

    for (int n = 0; n < (int)num_property_combinations[num_properties-1]; ++n)
    for (int i = 0; i < (int)sample_indices.size(); ++i)
    {
        if (dataset[sample_indices[i]][num_properties - 1] == "R" + int2String(n))
        {
            counts[n]++;
        }
    }

    for (int n = 0; n < (int)counts.size(); ++n)
    {
        if (counts[n] > max_count)
        {
            max_count = counts[n];
            index = n;
        }
    }

    counts.clear();

    return "R" + int2String(index);
}

int dataEngine::findMaximumGainPropertyIndex(std::vector<int>& sample_indices, std::vector<int>& property_indices)
{
    double max_gain = NEGATIVE_INF;
    double current_gain;
    int index = 0;
    for (int i = 0; i < (int)property_indices.size(); ++i)
    {
        current_gain = calculateGain(sample_indices, property_indices[i]);
        if (current_gain > max_gain)
        {
            max_gain = current_gain;
            index = property_indices[i];
        }
    }
    return index;
}

double dataEngine::calculateGain(std::vector<int>& subset_indices, int property_index)
{
    int num_indices = (int)subset_indices.size();
    double denominator = (double)num_indices;
    double initial_entropy = calculateRelativeEntropy(subset_indices);
    double ret_val_subentropies = 0.0;
    double numerator;
    std::vector<int> new_subset_indices;
    for (int i = 0; i < num_property_combinations[property_index]; ++i)
    {
        numerator = 0.0;
        for (int j = 0; j < num_indices; ++j)
        {
            if (dataset[subset_indices[j]][property_index] == int2String(property_index) + int2String(i))
            {
                numerator += 1.0;
                new_subset_indices.push_back(j);
            }
        }
        if (numerator > 0.0)
            ret_val_subentropies += (numerator / denominator)*calculateRelativeEntropy(new_subset_indices);
        new_subset_indices.clear();
    }
    return initial_entropy - ret_val_subentropies;
}

double dataEngine::calculateRelativeEntropy(std::vector<int>& subset_indices)
{
    int num_indices = (int)subset_indices.size();
    double denominator = (double)num_indices;
    double ret_val = 0.0;
    double numerator;
    for (int i = 0; i < num_property_combinations[num_properties - 1]; ++i)
    {
        numerator = 0.0;
        for (int j = 0; j < num_indices; ++j)
        {
            if (dataset[subset_indices[j]][num_properties - 1] == "R" + int2String(i))
            {
                numerator += 1.0;
            }
        }
        if (numerator > 0.0)
            ret_val += (numerator / denominator)*log2(numerator / denominator);
    }
    return -1.0 * ret_val;
}


int dataEngine::getNumSamples()
{
    return num_samples;
}

int dataEngine::getNumProperties()
{
    return num_properties;
}

std::string dataEngine::getAttributeName(int i)
{
    return attribute_names[i];
}

std::string dataEngine::getDataValue(int sample, int prop)
{
    return dataset[sample][prop];
}

bool dataEngine::randomlyPopulateDataset(int samp_int, int att_int, int propcombs_int)
{
    num_samples = samp_int;
    num_properties = att_int;

    dataset.resize(num_samples);

    for (int i = 0; i < num_samples; ++i)
        dataset[i].resize(num_properties);

    for (int i = 0; i < num_properties; ++i)
    {
        if (i < num_properties - 1)
        {
            attribute_names.push_back("A" + int2String(i));
        }
        else
        {
            attribute_names.push_back("AR");
        }
        num_property_combinations.push_back(rand() % (propcombs_int - 1) + 2);
    }

    for (int i = 0; i < num_samples; ++i)
    for (int j = 0; j < num_properties; ++j)
         dataset[i][j] = "NL";

    int fail_count = 0;

    for (int i = 0; i < num_samples; ++i)
    {
        do
        {
            fail_count++;
            for (int j = 0; j < num_properties; ++j)
            {
                if (j < num_properties - 1)
                    dataset[i][j] = int2String(j) + int2String(rand() % num_property_combinations[j]);
                else
                    dataset[i][j] = "R" + int2String(rand() % num_property_combinations[j]);
            }
        } while (sampleAlreadyExists(i) && fail_count < MAX_SAME_SAMPLE_THRESHOLD);

        if (fail_count >= MAX_SAME_SAMPLE_THRESHOLD)
        {
            std::cout << "WARNING: Unset node labels probably exist here: Decision tree incomplete/inaccurate.\n";
            break;
        }
    }

    while (dataset[(int)dataset.size() - 1][0] == "NL")
    {
        dataset.pop_back();
    }

    num_samples = (int)dataset.size();

    if (num_samples == 0 || num_properties == 0)
        return false;

    return true;
}

bool dataEngine::sampleAlreadyExists(int sample_index)
{
    bool ret_val_flag;

    if (sample_index == 0)
        return false;

    for (int i = 0; i < sample_index; ++i)
    {
        ret_val_flag = true;
        for (int j = 0; j < (int)dataset[i].size() - 1; ++j)
        {
            if (dataset[sample_index][j] != dataset[i][j])
            {
                ret_val_flag = false;
                break;
            }
        }
        if (ret_val_flag)
        {
            std::cout << "Sample already exists, generating new sample...\n";
            return true;
        }
    }

    return false;
}

class point
{
public:
    point();
    point(float,float);
    float x();
    float y();
    void setx(float);
    void sety(float);
    void setp(float,float);
    // assignment overload
    point operator=(point);
private:
    float x_val, y_val;
};

// default const.
point::point()
{
    x_val = y_val = 0.0f;
}

point::point(float xx, float yy)
{
    setp(xx, yy);
}

// set...
void point::setp(float xx, float yy)
{
    x_val = xx;
    y_val = yy;
}

// set y only
void point::sety(float yy)
{
    y_val = yy;
}

// set x only
void point::setx(float xx)
{
    x_val = xx;
}

// get y value of point3D.
float point::y()
{
    return y_val;
}

// get x value of point3D.
float point::x()
{
    return x_val;
}

// assignment overload
point point::operator=(point p)
{
    x_val = p.x();
    y_val = p.y();
    return *this;
}

bool inRange(point p, point maxp, point minp)
{
    return (p.x() >= minp.x() && p.y() >= minp.y() &&
            p.x() <= maxp.x() && p.y() <= maxp.y());
}

bool isAt(point a , point b)
{
    return (a.x() == b.x() && a.y() == b.y());
}

point addPoints(point a, point b)
{
    return point(a.x() + b.x(), a.y() + b.y());
}

class button
{
    public:
        button();
        button(point, point);
        point getLoc();
        point getArea();
    private:
        point loc;
        point area;
};

button::button()
{

}

button::button(point l, point a)
{
    loc = l;
    area = a;
}

point button::getLoc()
{
    return loc;
}

point button::getArea() 
{
    return area;
}

class slider
{
    public:
        slider();
        slider(point,point,bool);
        button *getButton1();
        button *getButton2();
        point getArea();
        point getVal();
        void setVal(point);
        bool getType();
    private:
        point val;
        point area;
        button b1, b2;
        bool slider_type;
};

slider::slider()
{

}

slider::slider(point v, point a, bool t)
{
    area = a;
    val = v;
    slider_type = t;
}

button *slider::getButton1()
{
    return &b1;
}

button *slider::getButton2()
{
    return &b2;
}

point slider::getArea()
{
    return area;
}

point slider::getVal()
{
    return val;
}

void slider::setVal(point v)
{
    val = v;
}

bool slider::getType()
{
    return slider_type;
}

class subwindowEngine
{
    public:
        subwindowEngine();
        subwindowEngine(point, point, point);
        void setTopLeft(point);
        void setTotalArea(point);
        slider *getUpDownSlider();
        slider *getLeftRightSlider();
        point getTopLeft();
        point getMinView();
        point getTotalArea();
    private:
        // todo: make a coord (or point) class to condense amount of fields
        point top_left;
        point max_view;
        point min_view;
        point total_area;
        slider up_down_slider;
        slider left_right_slider;
};

subwindowEngine::subwindowEngine()
{
    top_left = min_view = total_area = point(1.0f,1.0f);
}

subwindowEngine::subwindowEngine(point tl, point minv, point a)
{
    top_left = tl;
    min_view = minv;
    total_area = a;
}

void subwindowEngine::setTopLeft(point p)
{
    top_left = p;
}

point subwindowEngine::getTopLeft()
{
    return top_left;
}

void subwindowEngine::setTotalArea(point a)
{
    total_area = a;
}

point subwindowEngine::getTotalArea()
{
    return total_area;
}

point subwindowEngine::getMinView()
{
    return min_view;
}

slider *subwindowEngine::getUpDownSlider()
{
    return &up_down_slider;
}

slider *subwindowEngine::getLeftRightSlider()
{
    return &left_right_slider;
}

class displayEngine
{
    public:
        displayEngine();
        ~displayEngine();
        bool loadDisplayEngineSetWindow();
        GLFWwindow * loadDisplayEngine();
        GLFWwindow * getWindow();
    private:
        GLFWwindow * window;
};

displayEngine::displayEngine()
{
}

displayEngine::~displayEngine()
{
}

GLFWwindow * displayEngine::getWindow()
{
    return window;
}

GLFWwindow * displayEngine::loadDisplayEngine()
{
    if (!glfwInit())
    {
        std::cout << "Could not initialize GLFW3\n";
        return NULL;
    }

    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    GLFWwindow * create_window = glfwCreateWindow(AREA_WID, AREA_HGT, "Decision Tree Generator for Random Datasets", NULL, NULL);

    if (create_window == NULL)
    {
        std::cout << "Failed to open GLFW3 window.\n";
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(create_window);
    glfwSwapInterval(1);

    glfwSetInputMode(create_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    glEnable(GL_TEXTURE_2D);

    return create_window;
}

bool displayEngine::loadDisplayEngineSetWindow()
{
    window = loadDisplayEngine();
    if (window == NULL)
        return false;
    return true;
}

class texture
{
    public:
        texture();
        bool loadBMPTexture(const char*, GLint, GLint, GLint, bool, bool);
        GLuint getTextureBind();
    private:
        GLuint texture_bind;
};

texture::texture()
{
    texture_bind = 0;
}

bool texture::loadBMPTexture(const char * texturepath, GLint width, GLint height, GLint header_size, bool wrap, bool alpha)
{
    FILE * file;

    // open texture data
    if (fopen_s(&file, texturepath, "rb") == 0)
    {
        int image_size;

        int num_components;

        int dimensions = width * height;

        if (alpha)
            num_components = 4;
        else
            num_components = 3;

        image_size = dimensions * num_components;

        unsigned char * data = new unsigned char[image_size];
        fseek(file, header_size, SEEK_CUR);
        fread(data, 1, image_size, file);
        fclose(file);

        glGenTextures(1, &texture_bind);
        glBindTexture(GL_TEXTURE_2D, texture_bind);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap ? GL_REPEAT : GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap ? GL_REPEAT : GL_CLAMP);
        glTexImage2D(GL_TEXTURE_2D, 0, num_components, width, height, 0, (num_components == 4 ? GL_BGRA_EXT : GL_BGR_EXT), GL_UNSIGNED_BYTE, data);
        delete[] data;
        return true;
    }

    return false;
}

GLuint texture::getTextureBind()
{
    return texture_bind;
}

void renderCharacter(texture *, int, int, int);

void renderString(texture *, std::string, int, int);

bool checkButtonClick(GLFWwindow * window, subwindowEngine *swe)
{
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    y = AREA_HGT - y;

    if (inRange(point(x, y), point(TEXT_TILE_WIDTH + swe->getMinView().x(), swe->getMinView().y()), point(swe->getMinView().x(), swe->getMinView().y() - TEXT_TILE_HEIGHT)))
    {
        swe->setTopLeft(addPoints(swe->getTopLeft(), point(-5.0f, 0.0f)));
        return true;
    }
    else if (inRange(point(x, y), point(swe->getMinView().x()+swe->getTotalArea().x(), swe->getMinView().y()), point(swe->getMinView().x()+swe->getTotalArea().x() - TEXT_TILE_WIDTH, swe->getMinView().y() - TEXT_TILE_HEIGHT)))
    {
        swe->setTopLeft(addPoints(swe->getTopLeft(), point(5.0f, 0.0f)));
        return true;
    }
    else if (inRange(point(x, y), point(swe->getMinView().x() + swe->getTotalArea().x() + TEXT_TILE_WIDTH, swe->getMinView().y() + TEXT_TILE_HEIGHT), point(swe->getMinView().x() + swe->getTotalArea().x(), swe->getMinView().y())))
    {
        swe->setTopLeft(addPoints(swe->getTopLeft(), point(0.0f, 5.0f)));
        return true;
    }
    else if (inRange(point(x, y), point(swe->getMinView().x() + swe->getTotalArea().x() + TEXT_TILE_WIDTH, swe->getMinView().y() + swe->getTotalArea().y()), point(swe->getMinView().x() + swe->getTotalArea().x(), swe->getMinView().y()+swe->getTotalArea().y() - TEXT_TILE_HEIGHT)))
    {
        swe->setTopLeft(addPoints(swe->getTopLeft(), point(0.0f, -5.0f)));
        return true;
    }

    return false;
}

void drawCircle(int locx, int locy, int rad, bool leaf, bool is_root)
{
    glBegin(GL_POINTS);

    if (is_root)
        glColor3f(0.0f, 1.0f, 0.0f);
    else if (leaf)
        glColor3f(1.0f, 0.0f, 0.0f);
    else
        glColor3f(1.0f, 1.0f, 0.0f);

    double div;
    for (int offy = -rad; offy <= rad; ++offy)
    {
        div = sqrt((double)((rad*rad) - (offy*offy)));
        for (int offx = -(int)div; offx <= (int)div; ++offx)
        {
            glVertex2i(locx + offx, locy + offy);
        }
    }
    glEnd();
}

void drawLine(float r, float g, float b, float sx, float sy, float dx, float dy)
{
    glColor3f(r,g,b);
    glBegin(GL_LINES);
    glVertex2f(sx,sy);
    glVertex2f(dx,dy);
    glEnd();
}

void drawConnectionLines(node *parent, node *child)
{
    if (true)
    {
        drawLine(0.5f, 0.5f, 0.5f,(float)(parent->loc_data.x_coord) + 1.0f, ((float)parent->loc_data.y_coord), (float)(parent->loc_data.x_coord), ((float)parent->loc_data.y_coord) + 24.0f);
        drawLine(0.5f, 0.5f, 0.5f, (float)(parent->loc_data.x_coord) + 1.0f, ((float)parent->loc_data.y_coord) + 24.0f, (float)(child->loc_data.x_coord), ((float)parent->loc_data.y_coord) + 24.0f);
        drawLine(0.5f, 0.5f, 0.5f, (float)(child->loc_data.x_coord) + 1.0f, ((float)parent->loc_data.y_coord) + 24.0f, (float)(child->loc_data.x_coord), ((float)child->loc_data.y_coord));
    }
    else
    {
        drawLine(0.5f, 0.5f, 0.5f, (float)(parent->loc_data.x_coord) + 1.0f, ((float)parent->loc_data.y_coord), (float)(child->loc_data.x_coord), ((float)child->loc_data.y_coord));
    }
}

void drawDatasetValues(texture *font_bitmap, node *parent, node *child)
{
    float x_coord = child->loc_data.x_coord;
    float y_coord = child->loc_data.y_coord - 24.0f;
    glColor3f(1.0f, 1.0f, 1.0f);
    renderString(font_bitmap, child->stats.feedin_branch_value, (int)x_coord, (int)y_coord);
}

void renderAllNodesHelper(node *n)
{
    if (n->children == NULL)
        return;

    for (int i = 0; i < n->info.num_children; ++i)
    {
        drawConnectionLines(n, &(n->children[i]));
        drawCircle((int)n->children[i].loc_data.x_coord,
                   (int)n->children[i].loc_data.y_coord,
                        n->children[i].info.radius, n->children[i].children == NULL,false);
        renderAllNodesHelper(&(n->children[i]));
    }
}

void renderAllNodes(dataEngine *data_obj)
{
    drawCircle((int)data_obj->getRootNode()->loc_data.x_coord,
               (int)data_obj->getRootNode()->loc_data.y_coord,
               (int)data_obj->getRootNode()->info.radius, false, true);
    renderAllNodesHelper(data_obj->getRootNode());
}

void renderDataset(texture *font_bitmap, dataEngine *data_obj)
{
    glColor3f(1.0f, 1.0f, 1.0f);
    renderString(font_bitmap, "Sample dataset:", 15, 0);
    for (int i = 0; i < data_obj->getNumProperties(); ++i)
    {
        if (data_obj->getAttributeName(i) == data_obj->getRootNode()->stats.label)
            glColor3f(0.0f, 1.0f, 0.0f);
        else if (i == data_obj->getNumProperties() - 1)
            glColor3f(1.0f, 0.0f, 0.0f);
        else
            glColor3f(1.0f, 1.0f, 0.0f);
        renderString(font_bitmap, data_obj->getAttributeName(i), 15 + (int)(i * TEXT_TILE_WIDTH*5.0f),(3*TEXT_TILE_HEIGHT/2));
        for (int j = 0; j < data_obj->getNumSamples(); ++j)
        {
            if (i == data_obj->getNumProperties() - 1)
                glColor3f(1.0f, 0.0f, 0.0f);
            else
                glColor3f(1.0f, 1.0f, 1.0f);

            renderString(font_bitmap, data_obj->getDataValue(j, i) + " ", 15 + (int)(i * TEXT_TILE_WIDTH*5.0f),(int)((j + 2) * TEXT_TILE_HEIGHT*2.0f));
        }
    }
}

void renderAllTextHelper(texture *font_bitmap, node *n)
{
    if (n->info.is_root)
        glColor3f(0.0f, 1.0f, 0.0f);
    else if (n->children != NULL)
        glColor3f(1.0f, 1.0f, 0.0f);
    else
        glColor3f(1.0f, 0.0f, 0.0f);
    renderString(font_bitmap, n->stats.label, n->loc_data.x_coord+6.0, n->loc_data.y_coord-3.0);

    if (n->children == NULL)
        return;

    for (int i = 0; i < n->info.num_children; ++i)
    {
        drawDatasetValues(font_bitmap, n, &(n->children[i]));
        renderAllTextHelper(font_bitmap, &(n->children[i]));
    }
}

void renderAllText(texture *font_bitmap, dataEngine *data_obj)
{
    renderAllTextHelper(font_bitmap, data_obj->getRootNode());
}

void renderCharacter(texture *font_bitmap, int ascii, int xpos, int ypos)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindTexture(GL_TEXTURE_2D, font_bitmap->getTextureBind());
    glBegin(GL_QUADS);
    glTexCoord2f((((float)(ascii % (int)TEXT_BITMAP_COLS) * TEXT_TILE_WIDTH) + TEXT_TILE_WIDTH) / FONT_BITMAP_XSIZE,
                 1.0f - (((float)((int)(ascii / TEXT_BITMAP_COLS)) * TEXT_TILE_HEIGHT) + TEXT_TILE_HEIGHT) / FONT_BITMAP_YSIZE);
    glVertex2f((float)xpos + TEXT_TILE_WIDTH, (float)ypos + TEXT_TILE_HEIGHT);
    glTexCoord2f(((float)(ascii % (int)TEXT_BITMAP_COLS) * TEXT_TILE_WIDTH) / FONT_BITMAP_XSIZE,
                 1.0f - (((float)((int)(ascii / TEXT_BITMAP_COLS)) * TEXT_TILE_HEIGHT) + TEXT_TILE_HEIGHT) / FONT_BITMAP_YSIZE);
    glVertex2f((float)xpos, (float)ypos + TEXT_TILE_HEIGHT);
    glTexCoord2f(((float)(ascii % (int)TEXT_BITMAP_COLS) * TEXT_TILE_WIDTH) / FONT_BITMAP_XSIZE, 
                 1.0f - ((float)((int)(ascii / TEXT_BITMAP_COLS)) * TEXT_TILE_HEIGHT) / FONT_BITMAP_YSIZE);
    glVertex2f((float)xpos, (float)ypos);
    glTexCoord2f((((float)(ascii % (int)TEXT_BITMAP_COLS) * TEXT_TILE_WIDTH) + TEXT_TILE_WIDTH) / FONT_BITMAP_XSIZE,
                 1.0f - ((float)((int)(ascii / TEXT_BITMAP_COLS)) * TEXT_TILE_HEIGHT) / FONT_BITMAP_YSIZE);
    glVertex2f((float)xpos + TEXT_TILE_WIDTH, (float)ypos);
    glEnd();
    glDisable(GL_BLEND);
}

void renderString(texture *font_bitmap, std::string s, int x_pos, int y_pos)
{
    for (int i = 0; i < (int)s.size(); ++i)
    {
        renderCharacter(font_bitmap, (int)s[i], x_pos + i*(int)TEXT_TILE_WIDTH, y_pos);
    }
}

std::string double2String(double d)
{
    std::string s;
    std::stringstream ss;
    ss << d;
    s = ss.str();
    return s;
}

std::string int2String(int i)
{
    std::string s;
    std::stringstream ss;
    ss << i;
    s = ss.str();
    return s;
}

int string2Int(std::string s)
{
    int result = 0;
    int digit = 0;
    int factor = 1;
    int ssize = s.size();
    for (int i = 0; i < ssize; ++i)
    {
        if ((int)s[i] < 48 || (int)s[i] > 57)
        {
            return -1;
        }
    }
    for (int i = ssize - 1; i >= 0; --i)
    {
        digit = (int)s[ssize - i - 1] - 48;
        factor = 1;
        for (int j = 0; j < i; ++j)
        {
            factor *= 10;
        }
        result += (digit * factor);
    }
    return result;
}

void renderSliders(texture * font_bitmap, subwindowEngine *swe)
{
    glEnable(GL_TEXTURE_2D);

    glColor3f(1.0f, 1.0f, 1.0f);

    renderCharacter(font_bitmap, 17, (int)swe->getMinView().x(), (int)swe->getMinView().y() + (int)swe->getTotalArea().y() - TEXT_TILE_HEIGHT);
    renderCharacter(font_bitmap, 16, (int)swe->getMinView().x() + (int)swe->getTotalArea().x()- (int)TEXT_TILE_WIDTH, (int)swe->getMinView().y()+ (int)swe->getTotalArea().y() - TEXT_TILE_HEIGHT);
    renderCharacter(font_bitmap, 31, (int)swe->getMinView().x() + (int)swe->getTotalArea().x(), (int)swe->getMinView().y() + (int)swe->getTotalArea().y() - 2.0f*TEXT_TILE_HEIGHT);
    renderCharacter(font_bitmap, 30, (int)swe->getMinView().x() + (int)swe->getTotalArea().x(), (int)swe->getMinView().y() - TEXT_TILE_HEIGHT);

    glDisable(GL_TEXTURE_2D);

    drawLine(0.7f, 0.7f, 0.7f, swe->getMinView().x() + TEXT_TILE_WIDTH, swe->getMinView().y() + swe->getTotalArea().y() - TEXT_TILE_HEIGHT / 2.0f, swe->getMinView().x() + swe->getTotalArea().x() - TEXT_TILE_WIDTH, swe->getMinView().y() + swe->getTotalArea().y() - TEXT_TILE_HEIGHT / 2.0f);
    drawLine(0.7f, 0.7f, 0.7f, swe->getMinView().x() + swe->getTotalArea().x() + TEXT_TILE_WIDTH/2.0f, swe->getMinView().y() + swe->getTotalArea().y() - 2.0f*TEXT_TILE_HEIGHT, swe->getMinView().x() + swe->getTotalArea().x() + TEXT_TILE_WIDTH / 2.0f, swe->getMinView().y());
}

void calibrateSubwindow(subwindowEngine * swe, bool clear)
{
    glViewport((int)swe->getMinView().x(), (int)swe->getMinView().y(), (int)swe->getTotalArea().x(), (int)swe->getTotalArea().y());
    
    if (clear)
    {
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT);
    }

    glMatrixMode(GL_PROJECTION_MATRIX);
    glLoadIdentity();
    gluOrtho2D(swe->getTopLeft().x(), swe->getTopLeft().x() + swe->getTotalArea().x(), swe->getTopLeft().y() + swe->getTotalArea().y(), swe->getTopLeft().y());
}

class mainEngine
{
public:
    mainEngine();
    void runProgram();
    void promptInputParameters();
    void runComputations();
    void initDataSet();
    void initInputVectors();
    void initFontTexture();
    void initSubWindowObjects();
    void initDisplay();
    void reDisplay();
    void promptInput();
private:
    dataEngine data_obj;
    displayEngine display;
    subwindowEngine sub_window1;
    subwindowEngine sub_window2;
    std::vector<int> test_vec1;
    std::vector<int> test_vec2;
    texture font_bitmap;
    int samp_int;
    int att_int;
    int propcombs_int;
    bool mouse_clicked_flag1;// = false;
    bool re_display_flag;// = true;
    bool load_display_success;// = display.loadDisplayEngineSetWindow();
    bool ds_populate_success;
    bool load_font_success;
    bool hold_slider_flag;
};

mainEngine::mainEngine()
{
    att_int = DEFAULT_NUM_ATTS;
    samp_int = DEFAULT_NUM_SAMPS;
    propcombs_int = DEFAULT_NUM_PROPCOMBS;
    re_display_flag = true;
    load_display_success = false;
    ds_populate_success = false;
    load_font_success = false;
    mouse_clicked_flag1 = false;
    hold_slider_flag = false;
}

void mainEngine::runProgram()
{
    std::cout << "Decision Tree Generator for Random Datasets -- copyright Eric Wolfson 2016-2017\n\n";

    promptInputParameters();

    initDataSet();

    if (ds_populate_success)
    {
        initInputVectors();
        runComputations();
    }
    else
    {
        std::cout << "Failed to populate dataset! Press enter to exit...\n";
        std::cin.get();
        return;
    }

    initDisplay();

    if (load_display_success)
    {
        initFontTexture();
        if (load_font_success)
        {
            initSubWindowObjects();
            do
            {
                if (re_display_flag)
                {
                    reDisplay();
                }
                promptInput();
            } while (!glfwWindowShouldClose(display.getWindow()));
        }
        else
        {
            std::cout << "Failed to load font! Press enter to exit...\n";
            std::cin.get();
        }
        glfwDestroyWindow(display.getWindow());
        glfwTerminate();
    }
    else
    {
        std::cout << "Failed to load display window! Press enter to exit...\n";
        std::cin.get();
    }
}

// command line inputs needed before display rendering
void mainEngine::promptInputParameters()
{
    std::string att_str = "", samp_str = "", propcombs_str = "";

    std::cout << "Enter the number of attributes (" + int2String(DEFAULT_NUM_ATTS) + "-" + int2String(MAX_NUM_ATTS) + ")\n";
    std::getline(std::cin, att_str);
    att_int = string2Int(att_str);
    if (att_int < DEFAULT_NUM_ATTS || att_int > MAX_NUM_ATTS)
    {
        att_int = DEFAULT_NUM_ATTS;
        std::cout << "Out of range: defaulting attributes to " + int2String(DEFAULT_NUM_ATTS) + "...\n";
    }

    std::cout << "Enter the number of samples (" + int2String(DEFAULT_NUM_SAMPS) + "-" + int2String(MAX_NUM_SAMPS) + ")\n";
    std::getline(std::cin, samp_str);
    samp_int = string2Int(samp_str);
    if (samp_int < DEFAULT_NUM_SAMPS || samp_int > MAX_NUM_SAMPS)
    {
        samp_int = DEFAULT_NUM_SAMPS;
        std::cout << "Out of range: defaulting samples to " + int2String(DEFAULT_NUM_SAMPS) + "...\n";
    }

    std::cout << "Enter the maximum number of properties per attribute (" + int2String(DEFAULT_NUM_PROPCOMBS) + "-" + int2String(MAX_NUM_PROPCOMBS) + ")\n";
    std::getline(std::cin, propcombs_str);
    propcombs_int = string2Int(propcombs_str);
    if (propcombs_int < DEFAULT_NUM_PROPCOMBS || propcombs_int > MAX_NUM_PROPCOMBS)
    {
        propcombs_int = DEFAULT_NUM_PROPCOMBS;
        std::cout << "Out of range: defaulting max num property combinations to " + int2String(DEFAULT_NUM_PROPCOMBS) + "...\n";
    }
}

void mainEngine::initDisplay()
{
    std::cout << "Preparing display window...\n";
    load_display_success = display.loadDisplayEngineSetWindow();
}

void mainEngine::initDataSet()
{
    ds_populate_success = data_obj.randomlyPopulateDataset(samp_int, att_int, propcombs_int);
}

void mainEngine::initFontTexture()
{
    load_font_success = font_bitmap.loadBMPTexture("fontbmp.bmp", (int)FONT_BITMAP_XSIZE, (int)FONT_BITMAP_YSIZE, 122, true, true);
}

void mainEngine::initSubWindowObjects()
{
    sub_window1 = subwindowEngine(point(0.0f, 0.0f), point(0.0f, TEXT_TILE_WIDTH), point(AREA_WID - TEXT_TILE_WIDTH, AREA_HGT / 2.0f - TEXT_TILE_HEIGHT));
    sub_window2 = subwindowEngine(point(0.0f, 0.0f), point(0.0f, AREA_HGT / 2.0f + TEXT_TILE_WIDTH + 1.0f), point(AREA_WID - TEXT_TILE_WIDTH, AREA_HGT / 2.0f - TEXT_TILE_HEIGHT));
}

void mainEngine::runComputations()
{
    data_obj.initTree();
    std::cout << "Running ID3 Algorithm...\n";
    data_obj.ID3Algorithm(test_vec1, test_vec2, NULL);
    std::cout << "Running Node Positioning Algorithm...\n";
    data_obj.setupNodePositions();
}

void mainEngine::initInputVectors()
{
    if (ds_populate_success)
    {
        for (int i = 0; i < data_obj.getNumSamples(); ++i)
        {
            test_vec1.push_back(i);
        }
        for (int i = 0; i < data_obj.getNumProperties() - 1; ++i)
        {
            test_vec2.push_back(i);
        }
    }
}

void mainEngine::promptInput()
{
    glfwPollEvents();

    if (glfwGetKey(display.getWindow(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(display.getWindow(), GL_TRUE);

    if (glfwGetMouseButton(display.getWindow(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        hold_slider_flag = true;

    if (glfwGetMouseButton(display.getWindow(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
        hold_slider_flag = re_display_flag = false;

    if (hold_slider_flag)
    {
        if (checkButtonClick(display.getWindow(), &sub_window1) ||
            checkButtonClick(display.getWindow(), &sub_window2))
            hold_slider_flag = re_display_flag = true;
        else
            hold_slider_flag = re_display_flag = false;
    }
}

void mainEngine::reDisplay()
{
    calibrateSubwindow(&sub_window1, true);

    glEnable(GL_TEXTURE_2D);

    renderDataset(&font_bitmap, &data_obj);

    calibrateSubwindow(&sub_window2, false);

    glDisable(GL_TEXTURE_2D);

    renderAllNodes(&data_obj);

    glEnable(GL_TEXTURE_2D);

    renderAllText(&font_bitmap, &data_obj);

    glViewport(0, 0, AREA_WID, AREA_HGT);
    glMatrixMode(GL_PROJECTION_MATRIX);
    glLoadIdentity();
    gluOrtho2D(0, AREA_WID, AREA_HGT, 0);

    renderSliders(&font_bitmap, &sub_window1);

    renderSliders(&font_bitmap, &sub_window2);

    glfwSwapBuffers(display.getWindow());
}

int main(int argc, char *argv[])
{
    srand(time(NULL));
    mainEngine program_obj;
    program_obj.runProgram();
    return 0;
}
