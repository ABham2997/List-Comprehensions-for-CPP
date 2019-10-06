#ifndef LIST_COMPREHENSION_H
#define LIST_COMPREHENSION_H

#include<iostream>
#include<type_traits>
#include<cassert>
#include<vector>
#include<functional>
#include<algorithm>
#include<initializer_list>

#ifndef LISTCOMP_DISABLE_STD_CONTAINERS
#include<deque>
#include<list>
#include<forward_list>
#endif

#define _or ||
#define _and &&

namespace listcomp{

class placeholder;
template<auto> class trans;

template<auto P>
class pred{
public:
    decltype(P) predFunctor{P};

    pred(placeholder &_ph) {};
    pred(pred &) = delete;
    pred(pred &&other) = default;
    pred() = delete;
    pred &operator=(pred &) = delete;
    pred &operator=(pred &&) = delete;
};

namespace impl{

template<typename T>
using PredFunctor = std::function<bool(T)>;

template<typename T>
using ElseFunctor = std::function<T(T)>;

template <typename RT, typename Arg>
using TransFunctor = std::function<RT(Arg)>;

#define ADD_LIST_COMP_OPERATOR(TemplateClass,Typetag)\
operator TemplateClass<Typetag> () {\
    return TemplateClass<Typetag>(begin(), end());\
}\
\
template<typename TT, typename=std::void_t<decltype(TT(std::declval<Typetag>()))>>\
operator TemplateClass<TT> () {\
    return TemplateClass<TT>(begin(), end());\
}\

template<template<typename...> typename Cont, typename T, typename=void>
struct is_cont_impl : std::false_type{
};

template<template<typename...> typename Cont, typename T>
struct is_cont_impl<Cont, T, std::void_t<decltype(*(std::declval<Cont<T>>().begin())), decltype(*(std::declval<Cont<T>>().end()))>> :
    std::true_type{
};

template <template <typename...> typename Cont, typename T>
constexpr bool is_cont_v = is_cont_impl<Cont, T>::value;

#ifdef LISTCOMP_CONVERTABLES
template<typename UT, template<typename...> typename... Ts>
struct impl_oper : public impl_oper<UT,Ts>... {

};

template<typename UT, template<typename...> typename T>
struct impl_oper<UT,T> {
    virtual typename std::vector<UT>::iterator begin() = 0;
    virtual typename std::vector<UT>::iterator end() = 0;

    ADD_LIST_COMP_OPERATOR(T, UT);
};

template<typename T>
class implicit_convertable : public impl_oper<T, LISTCOMP_CONVERTABLES>{
#else
template<typename T>
class implicit_convertable{
#endif
    using Iter_t = typename std::vector<T>::iterator;
    private:
        std::optional<PredFunctor<T>> hasPred = std::nullopt;
        std::optional<ElseFunctor<T>> hasElse = std::nullopt;
        bool filterDone = false;

        implicit_convertable() : vec{} {};

    protected:
        std::vector<T> vec;

        void do_if_filter(){
            auto predFunctor = hasPred.value();
            vec.erase(std::remove_if(vec.begin(), vec.end(), [&](auto i) { return !predFunctor(i); }),vec.end());
        }

        void do_if_else_filter(){
            auto predFunctor = hasPred.value();
            auto elseFunctor = hasElse.value();
            for(auto &i:vec){
                if(!predFunctor(i)){
                    i = elseFunctor(i);
                }
            }
        }

        void do_filter(){
            if(filterDone) return; //filters already done or no other actions
            if(hasPred && !hasElse){ //if and no else
                do_if_filter();
            }
            else if(hasPred && hasElse){ //if and else
                do_if_else_filter();
            }
            else return;
            filterDone = true;
        }

    public:
        implicit_convertable(implicit_convertable<T>&& other, PredFunctor<T>&& predFunc) : vec(other.vec), 
                                                                                        hasPred{predFunc} {};
        implicit_convertable(implicit_convertable<T>&& other, ElseFunctor<T>&& elseFunc) : vec(other.vec),
                                                                hasPred{other.hasPred}, hasElse{elseFunc} {};

        template <typename TT>
        implicit_convertable(const TT &begin, const TT &end) : vec(begin, end){};

        implicit_convertable(std::vector<T> &&_vec) : vec(std::move(_vec)){};

        Iter_t begin() {
            do_filter();
            return vec.begin();
        }

        Iter_t end() {
            do_filter();
            return vec.end();
        }

        auto begin() const {
            do_filter();
            return vec.cbegin();
        }

        auto end () const {
            do_filter();
            return vec.cend();
        }

        operator T*(){
            return &vec[0];
        }

        template<typename TT, typename=std::void_t<decltype(TT(std::declval<T>()))>>
        operator TT*() {
            return &vec[0];
        }

        ADD_LIST_COMP_OPERATOR(std::vector, T);

#ifndef LISTCOMP_DISABLE_STD_CONTAINERS
        ADD_LIST_COMP_OPERATOR(std::list, T);

        ADD_LIST_COMP_OPERATOR(std::deque, T);

        ADD_LIST_COMP_OPERATOR(std::forward_list, T);
#endif
};

template<typename T>
class else_impl : public implicit_convertable<T>{
    public:
        using implicit_convertable<T>::implicit_convertable;
};

template<typename T>
class if_impl : public implicit_convertable<T>{
    public:
        using implicit_convertable<T>::implicit_convertable;

        template<auto F>
        else_impl<T> _else(trans<F>&& transFunc){
            ElseFunctor<T> elseFunctor = F;
            return else_impl<T>(std::move(*this), std::move(elseFunctor));
        }

        else_impl<T> _else(placeholder&){
            ElseFunctor<T> elseFunctor = [](auto val) { return val; };
            return else_impl<T>(std::move(*this), std::move(elseFunctor));
        }

        else_impl<T> _else(const T& val){
            ElseFunctor<T> elseFunctor = [&] (auto arg) { return val; };
            return else_impl<T>(std::move(*this), std::move(elseFunctor));
        }
};

template<typename T>
class in_impl : public implicit_convertable<T>{
    public:
        using implicit_convertable<T>::implicit_convertable;

        template<auto P>
        if_impl<T> _if(pred<P>&& predicate){
            PredFunctor<T> predFunctor = predicate.predFunctor;
            return if_impl<T>(std::move(*this), std::move(predFunctor));
        }

        if_impl<T> _if(placeholder&) {
            PredFunctor<T> predFunctor = [](auto val) { return true; };
            return if_impl<T>(std::move(*this), std::move(predFunctor));
        }
};

template<auto F>
class for_impl{
    public:
        for_impl() = default;

        template <template<typename> typename Cont, typename T>
        auto _in(const Cont<T> &container){
            static_assert(is_cont_v<Cont,T>, "argument to _in is not a container type");
            using TT = decltype(F(std::declval<T>()));
            std::vector<TT> vec;
            for(const auto &i:container){
                vec.push_back(F(i));
            }
            return in_impl<TT>(std::move(vec));
        }

        template <typename T>
        auto _in(std::initializer_list<T> &&container){
            using TT = decltype(F(std::declval<T>()));
            std::vector<TT> vec;
            for(const auto &i:container){
                vec.push_back(F(i));
            }
            return in_impl<TT>(std::move(vec));
        }
};

template<>
class for_impl<0>{
    public:
        for_impl() = default;

        template <template<typename> typename Cont, typename T>
        auto _in(const Cont<T> &container){
            static_assert(is_cont_v<Cont,T>, "argument to _in is not a container type");
            return in_impl<T>(container.begin(), container.end());
        }

        template <typename T>
        auto _in(std::initializer_list<T> &&container){
            return in_impl<T>(std::vector<T>(container));
        }
};

} //namespace impl


class placeholder{
    private:
        const int id;

        inline static char inst_cnt = 0;

        placeholder(int i) : id{i} { inst_cnt++; }
        placeholder(placeholder&& other) : id{other.id} {};

        template <auto F> friend class trans;

    public:
        placeholder() : id{inst_cnt} { inst_cnt++; }
        placeholder(placeholder &) : placeholder{} {};
        placeholder &operator=(placeholder &) = delete;
        placeholder &operator=(placeholder &&) = delete;

        const int get_id() const {
            return id;
        }

        placeholder operator,(const placeholder &other){
            return std::move(placeholder{id+other.id});
        }

        impl::for_impl<0> _for(placeholder&){
            return impl::for_impl<0>{};
        }

}_i,_j,_k;

template<auto F>
class trans{
    using Func=decltype(F);
    private:
        placeholder &ph;
        Func functor{F};

    public:
        trans(placeholder &_ph) : ph{_ph} {};
        trans() = delete;
        trans(trans &) = delete;
        trans(trans &&) = delete;
        trans &operator=(trans &) = delete;
        trans &operator=(trans &&) = delete;

        impl::for_impl<F> _for(placeholder&) {
            return impl::for_impl<F>{};
        }
};

placeholder new_placeholder(){
    return placeholder{};
}

} //namespace listcomp

#endif