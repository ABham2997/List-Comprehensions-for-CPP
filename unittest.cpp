#include<deque>
#include<list>
#include<vector>
#include<cmath>

template<typename T>
struct my_iter{
    std::vector<T> vec;
    template <typename TT>
    my_iter(TT begin, TT end) : vec(begin, end){};

    auto begin() {
        return vec.begin();
    }

    auto end() {
        return vec.end();
    }
};

template<typename T>
struct my_iter_2{
    std::vector<T> vec;
    template <typename TT>
    my_iter_2(TT begin, TT end) : vec(begin, end){};

    auto begin() {
        return vec.begin();
    }

    auto end() {
        return vec.end();
    }  
};

#define LISTCOMP_CONVERTABLES my_iter,my_iter_2

#include "list_comp.h"

#include<iostream>
#include<memory>

int functor(int i){
    return i * 2;
}

double sqr(double i){
    return i * i;
}

bool is_even(int i){
    return i % 2 == 0;
}

struct funct{
    int i;
    int operator() (int i) const {
        return i * i;
    }
};

char to_char(int i){
    return static_cast<int>(i + int('a') - 1);
}

int is_odd(int i){
    return i%2!=0;
}

double msqrt(int i){
    return std::sqrt(i);
}

bool is_sqr(int i){
    return std::sqrt(i) == int(std::sqrt(i));
}

int plus_thou(int i){
    return i + 1000;
}

int main(){
    using namespace listcomp;

    std::vector<int> v{1, 2, 3, 4, 5};

    placeholder i,j;

    std::vector<char> cv = trans<to_char>(i)._for(i)._in(v)._if(pred<is_odd>(i))._else('a');

    std::vector<int> av = trans<sqr>(i)._for(i)._in(v);

    std::deque<int> vv = trans<sqr>(i)._for(i)._in(v)._if(pred<is_even>(i));

    std::list<int> vvv = j._for(j)._in(v);

    my_iter<float> myit = i._for(i)._in({10,20,30,40,50})._if(40.0<=i _and i>=20 _or i==10 _or pred<is_sqr>(i))._else(0);

    my_iter_2<double> myit2 = i._for(i)._in({0, 2, 3, 4, 5, 9, 15, 16, 20, 25})._if(i)._else(i+100);

    for(const auto& ii: cv){
        std::cout << ii << ", ";
    }

    std::cout << '\n';

    for(const auto& ii:myit2){
        std::cout << ii << ", ";
    }

    std::cout << '\n';

    return 0;
}