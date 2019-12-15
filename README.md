# List Comprehensions for C++

Have you ever admired the flexibility, power and intuitiveness of list comprehensions in python and wanted to emulate the same feature in c++ without writing a brand new c++ parser? Well, you still can't, but you can get pretty close with List Comprehensions for C++. Observe:

```c++
#include<vector>
#include<deque>
#include<list>

#include "pylistcomp.h"

int main(){
    using namespace pylistcomp;
    
    std::vector<int> example1 = 
            _i._for(_i)._in({1,2,3,4,5}); //_i is a placeholder variable already defined in namespace listcomp

    placeholder number; //you can also declare your own placeholders
    std::vector<float> numbers{25.5,32.3,11.1, 123.4, 5.5};
    std::deque<double> example2{ number._for(number)._in(numbers)._if(number<=30) };

    placeholder i;
    std::list<double> example3 = 
            i._for(i)._in({1,2,4,6,9,12,16,20,25})._if(10<=i _and i>=20 _or i==6)._else(i*i);

    std::forward_list<short> example4 = 
            i._for(i)._in(_range(5,50))._if(i%5==0)._else(0); //_range returns a lightweight iterable object, similar to python ranges

    return 0;
}
```

\
By default list comprehensions can be used to construct std::vectors, std::lists, std::deques and std::forward_lists. However, you can add support for your own container template using the LISTCOMP_CONVERTABLES macro. The only constraints: your container must be constructible from two std::iterator objects and must have at most, one non-default template type parameter. To use the macro, #define LISTCOMP_CONVERTABLES as your list of container templates, **before** #include-ing the pylistcomp file. Demonstration:
```c++
#include<vector>
#include<iterator>
#include<allocator>
#include<list>

template<typename T>
class MyContainerA{
    private:
        std::vector<T> data;

    public:
        template<typename Tag>
        MyContainerA(std::iterator<Tag,T> begin, std::iterator<Tag,T> end) : data(begin,end) {};

        //...//
}; 

template<typename T, typename allocator=std::allocator<T>> //only one non-default template type-parameter
class MyContainerB{
    private:
        std::list<T,allocator> data;

    public:
        template<typename TT>
        MyContainerB(TT begin, TT end) : data(begin, end) {};

        //...//
};

//list of container templates
#define LISTCOMP_CONVERTABLES MyContainerA,MyContainerB 

//macro defined before pylistcomp file included
#include"pylistcomp.h"

int main(){
    using namespace pylistcomp;

    std::vector<int> someData{20,0,0,100,11,121,0,0,13};
    MyContainerA<int> example1 = _i._for(_i)._in(someData)._if(i);

    placeholder letter;
    MyContainerB<char> example2 = 
            letter._for(letter)._in("this is\na string")._if(letter!='\n' _or letter!='\0');

    return 0;
}

```

\
Similarly, you can iterate through any container that has non-static public member methods 'begin' and 'end' that return std::iterators, like so:
```c++
#include<vector>
#include<string>
#include<initializer_list>
#include"pylistcomp.h"

template<typename T>
class MyIterableContainer{
    private:
        std::vector<T> data;

    public:
        MyIterableContainer(const std::initializer_list<T>& _data) : data(_data) {};

        std::vector<T>::iterator begin() {
            return data.begin();
        }

        std::vector<T>::iterator end() {
            return data.end();
        }

        auto begin() const {
            return data.cbegin();
        }

        auto end() const {
            return data.cend();
        }
};

int main(){
    using namespace placeholders;
    
    MyIterableContainer<std::string> words{"this","is","some","data"};

    placeholder word;
    std::vector<std::string> example1 = word._for(word)._in(words);

    return 0;
}
```

\
If you have want to remove list comprehension compatibility with std::forward_lists, std::deques and std::lists (if, for e.g., you're not using those containers and want to avoid compile-time penalties), simple #define LISTCOMP_DISABLE_STD_CONTAINERS before #include-ing pylistcomp.h. Compatibility with std::vectors is default behaviour and cannot be removed:
```c++
#define LISTCOMP_DISABLE_STD_CONTAINERS

#include "pylistcomp.h"

#include<deque>
#include<list>

int main(){
    using namespace pylistcomp;

    std::vector<int> example1 = 
        _i._for(_i)._in({5,10,15,20}); //OK

    std::list<float> example2 =
        _i._for(_i)._in({1.5, 3.5, 7.5}); //ERROR    

    std::deque<char> example3 =
        _i._for(_i)._in({'a','c','e','d'}); //EROR

    return 0;
}
```

\
The namespace pylistcomp also has an lightweight iterable object, _range, that behaves similarly to range generators in python. It can be used to construct std::vectors, std::deques, std::lists and std::forward_iterators, or iterated through in range-based for-loops. It can also deal with negative numbers, doubles and floats. 
```c++
#include<vector>
#include<list>
#include<deque>
#include<forward_list>

#include"pylistcomp.h"

int main(){
    using namespace pylistcomp;

    std::vector<int> example1 = _range(10); //create vector from every integer between 0 and 10

    std::list<double> example2 = _range(-5,20); //create list from every double between -5 and 20

    std::deque<float> example3 = _range(2.5,30,1.5); //create deque from numbers between 2.5 and 30, in intervals of 1.5    

    std::forward_list<long> example4 = _range(-1000,-10000,-1000); //create forward_list from numbers between -1000 and -10000 in intervals of -1000

    for(auto i:_range(27,39)){
        //...//
    }

    for(auto i:_range(5,120,20)){
        //...//
    }

    return 0;
}

```
