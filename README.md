# List Comprehensions for C++

Have you ever admired the flexibility, power and intuitiveness of list comprehensions in python and wanted to emulate the same feature in c++ without writing a brand new c++ parser? Well, you still can't, but you can get pretty close with List Comprehensions for C++. Observe:

```c++
#include<vector>
#include<deque>
#include<list>

#include "pylistcomp.h"

int main(){
    using namespace pylistcomp;
    
    std::vector<int> example1 = _i._for(_i)._in({1,2,3,4,5}); //_i is a placeholder variable already defined in namespace listcomp

    placeholder number; //you can also declare your own placeholders
    std::vector<float> numbers{25.5,32.3,11.1, 123.4, 5.5};
    std::deque<double> example2{ number._for(number)._in(numbers)._if(i<=30) };

    placeholder i;
    std::list<double> example3 = i._for(i)._in({1,2,4,6,9,12,16,20,25})._if(10<=i _and i>=20 _or i==6)._else(i*i);

    std::forward_list<short> example4 = i._for(i)._in(_range(5,50))._if(i%5==0)._else(0); //_range returns a lightweight iterable object, similar to python ranges

    return 0;
}
```

\
By default list comprehensions can be used to construct std::vectors, std::lists, std::deques and std::forward_lists. However, you can add support for your own container template using the LISTCOMP_CONVERTABLES macro. The only constraints: your container must be constructible from two std::iterator objects and must take only one compulsory template parameter. Demonstration:
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

#define LISTCOMP_CONVERTABLES MyContainerA,MyContainerB

#include"pylistcomp.h"

int main(){
    using namespace pylistcomp;

    std::vector<int> someData{20,0,0,100,11,121,0,0,13};
    MyContainerA<int> example1 = _i._for(_i)._in(someData)._if(i);

    placeholder letter;
    MyContainerB<char> example2 = letter._for(letter)._in({'a','c','\n','\0','e'})._if(letter!='\n' _or letter!='\0');

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
    
    MyIterableContainer<string> words{"this","is","some","data"};

    placeholder word;
    std::vector<string> example1 = word._for(word)._in(container);

    return 0;
}
```
