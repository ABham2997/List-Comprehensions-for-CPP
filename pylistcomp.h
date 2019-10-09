#ifndef LISTCOMP_LIST_COMPREHENSION_H
#define LISTCOMP_LIST_COMPREHENSION_H

#include<type_traits>
#include<functional>
#include<algorithm>
#include<initializer_list>
#include<iterator>

#ifndef LISTCOMP_DISABLE_STD_CONTAINERS
#include<vector>
#include<deque>
#include<list>
#include<forward_list>
#endif

#ifndef LISTCOMP_DISABLE_OR_AND_NOT
#define _or ||
#define _and &&
#define _not !
#endif

namespace pylistcomp{

class placeholder;
template<auto> class trans;

namespace impl{

template<typename T>
using PredFunctor = std::function<bool(const T&)>;

template<typename InT, typename OutT>
using ElseFunctor = std::function<OutT(InT)>;

template <typename InT, typename OutT>
using TransFunctor = std::function<OutT(InT)>;

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
struct is_cont_impl<Cont, T, std::void_t<decltype(std::vector<T>((std::declval<Cont<T>>().begin()),(std::declval<Cont<T>>().end())))>> 
: std::true_type{
};

template <template <typename...> typename Cont, typename T>
constexpr bool is_cont_v = is_cont_impl<Cont, T>::value;

template <typename, typename, typename>
class implicit_convertable;

template<typename InT,typename OuT,typename=void>
struct is_constructible : std::false_type{
};

template<typename InT, typename OutT>
struct is_constructible<InT,OutT,std::void_t<decltype(OutT(std::declval<InT>()))>> : std::true_type{
};

template <typename InT, typename OutT>
constexpr bool is_cons_or_same_v = std::is_same_v<InT,OutT> || is_constructible<InT, OutT>::value;

template<typename T, typename...Ts>
struct head_of{
    using type = T;
};

template<typename T>
struct head_of<T>{
    using type = T;
};

template <typename... Ts>
struct function_ptr : std::false_type {
};

template<typename RT>
struct function_ptr<RT(*)()> : std::true_type{
    using ReturnType = RT;
    using ArgType = void;
    static constexpr int ArgSize = 0;
};

template<typename RT, typename...Ts>
struct function_ptr<RT(*)(Ts...)> : std::true_type{
    using ReturnType = RT;
    using ArgType = typename head_of<Ts...>::type;
    static constexpr int ArgSize = sizeof...(Ts);
};

template<typename InT, typename OutT, typename Iterator>
class iterator_underlying_t : public std::iterator<std::forward_iterator_tag, OutT>{
    private:
        void get_next(){
            if (enclosing->hasPred && !(enclosing->hasElse)) {
                PredFunctor<InT> predFunctor = enclosing->hasPred;
                while (iter != end && !predFunctor(*iter)){
                    iter++;
                }
            }
        }

    protected:
        Iterator iter;
        Iterator end;
        implicit_convertable<InT,OutT,Iterator> *enclosing;

    public:
        iterator_underlying_t &operator=(const iterator_underlying_t &other) = default;
        iterator_underlying_t(const iterator_underlying_t &other) = default;
        iterator_underlying_t(const Iterator &_iter, const Iterator &_end, implicit_convertable<InT,OutT,Iterator>* _enclosing) : 
        iter{_iter}, end{_end}, enclosing{_enclosing} {
            get_next();
        };        

};

template<typename InT, typename OutT, typename Iterator, typename=void>
class iterator_deref : public iterator_underlying_t<InT,OutT,Iterator> {
    private:
        OutT get_val(){
            TransFunctor<InT,OutT> transFunctor = this->enclosing->hasTrans;
            if(this->enclosing->hasElse){
                PredFunctor<InT> predFunctor = this->enclosing->hasPred;
                ElseFunctor<InT, OutT> elseFunctor = this->enclosing->hasElse;
                return predFunctor(*(this->iter)) ? transFunctor(*(this->iter)) : elseFunctor(*(this->iter));
            }
            return transFunctor(*(this->iter));
        }

    public:
        using iterator_underlying_t<InT, OutT,Iterator>::iterator_underlying_t;

        auto operator*(){
            return get_val();
        }
};

template<typename InT, typename OutT, typename Iterator>
class iterator_deref<InT, OutT, Iterator, std::enable_if_t<is_cons_or_same_v<InT,OutT>>> : public iterator_underlying_t<InT,OutT,Iterator> {  
    private:
        OutT get_val(){
            if(!(this->enclosing->hasTrans) && !(this->enclosing->hasElse)){
                return *(this->iter);
            }
            else if (this->enclosing->hasTrans && !(this->enclosing->hasElse)) {
                TransFunctor<InT,OutT> transFunctor = this->enclosing->hasTrans;
                return transFunctor(*(this->iter));
            } 
            else if(this->enclosing->hasTrans && this->enclosing->hasElse){
                TransFunctor<InT,OutT> transFunctor = this->enclosing->hasTrans;
                PredFunctor<InT> predFunctor = this->enclosing->hasPred;
                ElseFunctor<InT,OutT> elseFunctor = this->enclosing->hasElse;
                if(predFunctor(*(this->iter))){
                    return transFunctor(*(this->iter));
                } 
                else {
                    return elseFunctor(*(this->iter));
                }
            } 
            else if(!(this->enclosing->hasTrans) && this->enclosing->hasElse){
                PredFunctor<InT> predFunctor = this->enclosing->hasPred;
                ElseFunctor<InT,OutT> elseFunctor = this->enclosing->hasElse;
                if(predFunctor(*(this->iter))){
                    return *(this->iter);
                } 
                else {
                    return elseFunctor(*(this->iter));
                }
            }
            return *(this->iter);
        }

    public:
        using iterator_underlying_t<InT, OutT, Iterator>::iterator_underlying_t;

        OutT operator*(){
            return get_val();
        }
};

template<typename InT, typename OutT, typename Iterator>
class iterator : public iterator_deref<InT,OutT,Iterator> {
    public:
        using iterator_deref<InT, OutT, Iterator>::iterator_deref;

        iterator &operator++(){
            (*this) = iterator(++(this->iter), this->end, this->enclosing);
            return *this;
        }

        iterator operator++(int){
            iterator res(*this);
            ++(*this);
            return res;
        }
        iterator &operator--(){
            (*this) = iterator(--(this->iter), this->end, this->enclosing);
            return *this;
        }

        iterator operator--(int){
            iterator res(*this);
            --(*this);
            return res;
        }

        bool operator==(const iterator& other){
            return this->iter == other.iter;
        }

        bool operator!=(const iterator& other){
            return this->iter != other.iter;
        }
};

struct PredFlag{
} pred_flag;

struct ElseFlag{
} else_flag;

struct TransFlag{
} trans_flag;

#ifdef LISTCOMP_CONVERTABLES
template<typename InT, typename OutT, typename Iterator, template<typename...> typename... Ts>
struct impl_oper : public impl_oper<InT,OutT,Iterator,Ts>... {

};

template<typename InT, typename OutT, typename Iterator, template<typename...> typename T>
struct impl_oper<InT,OutT,Iterator,T> {
    virtual iterator<InT,OutT,Iterator> begin() = 0;
    virtual iterator<InT,OutT,Iterator> end() = 0;

    ADD_LIST_COMP_OPERATOR(T, OutT);
};

template<typename InT, typename OutT, typename Iterator>
class implicit_convertable : public impl_oper<InT, OutT, Iterator, LISTCOMP_CONVERTABLES>{
#else
template<typename InT, typename OutT, typename Iterator>
class implicit_convertable{
#endif
    private:
        Iterator start;
        Iterator finish;
        PredFunctor<InT> hasPred = nullptr;
        ElseFunctor<InT,OutT> hasElse = nullptr;
        TransFunctor<InT,OutT> hasTrans = nullptr;

        template<typename,typename,typename> friend class iterator_underlying_t;
        template<typename,typename,typename,typename> friend class iterator_deref;

    public:
        implicit_convertable(implicit_convertable&& other, PredFunctor<InT>&& predFunc, PredFlag&) : 
            start{other.start}, finish{other.finish}, hasTrans{other.hasTrans}, hasPred{predFunc} {};
        
        implicit_convertable(implicit_convertable&& other, ElseFunctor<InT,OutT>&& elseFunc, ElseFlag&) : 
            start{other.start}, finish{other.finish}, hasTrans{other.hasTrans}, hasPred{other.hasPred}, hasElse{elseFunc} {};

        template <typename TT>
        implicit_convertable(const TT &begin, const TT &end) : start{begin}, finish{end} {};

        template <typename TT>
        implicit_convertable(const TT &begin, const TT &end, TransFunctor<InT,OutT>&& trans) : start{begin}, finish{end}, hasTrans{trans} {};

        iterator<InT,OutT,Iterator> begin() {
            return iterator<InT,OutT,Iterator>(start,finish,this);
        }

        iterator<InT,OutT,Iterator> end() {
            return iterator<InT,OutT,Iterator>(finish,finish,this);
        }

#ifndef LISTCOMP_DISABLE_STD_CONTAINERS
        ADD_LIST_COMP_OPERATOR(std::vector, OutT);

        ADD_LIST_COMP_OPERATOR(std::list, OutT);

        ADD_LIST_COMP_OPERATOR(std::deque, OutT);

        ADD_LIST_COMP_OPERATOR(std::forward_list, OutT);
#endif
};

template<typename InT, typename OutT, typename Iterator>
class else_impl : public implicit_convertable<InT,OutT,Iterator>{
    public:
        using implicit_convertable<InT,OutT,Iterator>::implicit_convertable;
};

enum class oper_flag{
    mult,div,add,sub,mod,rmult,rdiv,radd,rsub,rmod
};

template<typename InT>
class proxy_trans{
    private:
        ElseFunctor<InT,InT> elseFunc;

        template<typename TT> friend class proxy_trans;

    public:
        auto get_else() const {
            return elseFunc;
        }

        proxy_trans(oper_flag flag, const InT& value){
            switch(flag){
                case oper_flag::mult:
                    elseFunc = [&](const auto& arg) { return arg * value; };
                    break;
                case oper_flag::div:
                    elseFunc = [&](const auto& arg) { return arg / value; };
                    break;
                case oper_flag::add:
                    elseFunc = [&](const auto& arg) { return arg + value; };
                    break;
                case oper_flag::sub:
                    elseFunc = [&](const auto& arg) { return arg - value; };
                    break;
                case oper_flag::mod:
                    elseFunc = [&](const auto& arg) { return arg % value; };
                    break;
                case oper_flag::rmult:
                    elseFunc = [&](const auto& arg) { return value * arg; };
                    break;
                case oper_flag::rdiv:
                    elseFunc = [&](const auto& arg) { return value / arg; };
                    break;
                case oper_flag::radd:
                    elseFunc = [&](const auto& arg) { return value + arg; };
                    break;
                case oper_flag::rsub:
                    elseFunc = [&](const auto& arg) { return value - arg; };
                    break;
                case oper_flag::rmod:
                    elseFunc = [&](const auto& arg) { return value % arg; };
                    break;
            }
        } 
};

template<typename InT, typename OutT, typename Iterator>
class if_impl : public implicit_convertable<InT,OutT,Iterator>{
    public:
        using implicit_convertable<InT,OutT,Iterator>::implicit_convertable;

        template<auto F>
        else_impl<InT,OutT,Iterator> _else(trans<F>&&){
            static_assert(is_cons_or_same_v<typename function_ptr<decltype(F)>::ReturnType,OutT>);
            ElseFunctor<InT, OutT> elseFunctor = F;
            return else_impl<InT, OutT, Iterator>(std::move(*this), std::move(elseFunctor),else_flag);
        }

        else_impl<InT,OutT,Iterator> _else(placeholder&){
            ElseFunctor<InT,OutT> elseFunctor = [&](const auto &arg) { return arg; };
            return else_impl<InT,OutT,Iterator>(std::move(*this), std::move(elseFunctor),else_flag);
        }

        else_impl<InT,OutT,Iterator> _else(const OutT& val){
            ElseFunctor<InT,OutT> elseFunctor = [&] (const auto& arg) { return val; };
            return else_impl<InT,OutT,Iterator>(std::move(*this), std::move(elseFunctor),else_flag);
        }

        else_impl<InT,OutT,Iterator> _else(proxy_trans<InT>&& proxy){
            return else_impl<InT,OutT,Iterator>(std::move(*this), proxy.get_else(),else_flag);
        }

        else_impl<InT,OutT,Iterator> _else(ElseFunctor<InT,OutT> elseFunctor){
            return else_impl<InT,OutT,Iterator>(std::move(*this), std::move(elseFunctor), else_flag);
        }
};

enum class bool_flag{
    equals, nequals, lthan, gthan, lthaneq, gthaneq, requals, rnequals, rlthan, rgthan, rlthaneq, rgthaneq
};

template<auto P>
class pred{
    public:
        pred(placeholder &) {
            using FuncSpec = function_ptr<decltype(P)>;
            static_assert(FuncSpec::value, "only function pointers can be passed as template arguments to pred");
            static_assert(FuncSpec::ArgSize == 1, "pred functions must take exactly one argument");
            static_assert(!std::is_same_v<typename FuncSpec::ArgType, void>, "pred functions must not take void-type arguments");
            static_assert(is_cons_or_same_v<typename FuncSpec::ReturnType, bool>, "pred functions must have bool or bool convertable return-type");            
        };
        pred(pred &) = delete;
        pred(pred &&other) = default;
        pred() = delete;
        pred &operator=(pred &) = delete;
        pred &operator=(pred &&) = delete;

};

template<typename T>
class proxy_bool{
    private:
        proxy_bool() = delete;
        PredFunctor<T> predFunc;

        template<typename TT> friend class proxy_bool;

    public:
        auto get_pred() const {
            return predFunc;
        }

        proxy_bool(bool_flag flag, const T& value, bool negative=false){
            switch(flag){
                case bool_flag::equals:
                    predFunc = [&,negative](auto arg) { return negative ? !(arg==value) : arg == value; };
                    break;
                case bool_flag::nequals:
                    predFunc = [&,negative](auto arg) { return negative ? !(arg!=value) : arg != value; };
                    break;
                case bool_flag::lthan:
                    predFunc = [&,negative](auto arg) { return negative ? !(arg<value) : arg < value; };
                    break;
                case bool_flag::gthan:
                    predFunc = [&,negative](auto arg) { return negative ? !(arg>value) : arg > value; };
                    break;
                case bool_flag::lthaneq:
                    predFunc = [&,negative](auto arg) { return negative ? !(arg<=value) : arg <= value; };
                    break;
                case bool_flag::gthaneq:
                    predFunc = [&,negative](auto arg) { return negative ? !(arg>=value) : arg >= value; };
                    break;
                case bool_flag::requals:
                    predFunc = [&](auto arg) { return value == arg; };
                    break;
                case bool_flag::rnequals:
                    predFunc = [&](auto arg) { return value != arg; };
                    break;
                case bool_flag::rlthan:
                    predFunc = [&](auto arg) { return value < arg; };
                    break;
                case bool_flag::rgthan:
                    predFunc = [&](auto arg) { return value > arg; };
                    break;
                case bool_flag::rlthaneq:
                    predFunc = [&](auto arg) { return value <= arg; };
                    break;
                case bool_flag::rgthaneq:
                    predFunc = [&](auto arg) { return value >= arg; };
                    break;
            }
        }

        proxy_bool(const PredFunctor<T> &_predFunc) : predFunc{_predFunc} {};

        template<typename TT>
        proxy_bool operator&&(const proxy_bool<TT>& other){
            return PredFunctor<T>{ [=](const auto& arg) { return predFunc(arg) && other.predFunc(arg); } };
        }

        template<auto P>
        proxy_bool operator&&(const pred<P>&){
            return PredFunctor<T>{[=](const auto &arg) { return P(arg) && predFunc(arg); }};
        }

        template<typename TT>
        proxy_bool operator||(const proxy_bool<TT>& other){
            return PredFunctor<T>{ [=](const auto& arg) { return predFunc(arg) || other.predFunc(arg); } };
        }

        template<auto P>
        proxy_bool operator||(const pred<P>&){
            return PredFunctor<T>{[=](const auto &arg) { return predFunc(arg) || P(arg); }};
        }

        proxy_bool operator!(){
            return PredFunctor<T>{[=](const auto &arg) { return !predFunc(arg); }};
        }
};

class not_proxy_bool{
    public:
        not_proxy_bool() = default;

        template<typename T>
        proxy_bool<T> operator==(T&& value){
            return proxy_bool<T>(bool_flag::equals, std::forward<T>(value), true);
        }

        template<typename T>
        proxy_bool<T> operator!=(T&& value){
            return proxy_bool<T>(bool_flag::nequals, std::forward<T>(value), true);
        }

        template<typename T>
        proxy_bool<T> operator<(T&& value){
            return proxy_bool<T>(bool_flag::lthan, std::forward<T>(value), true);
        }

        template<typename T>
        proxy_bool<T> operator>(T&& value){
            return proxy_bool<T>(bool_flag::gthan, std::forward<T>(value), true);
        }

        template<typename T>
        proxy_bool<T> operator<=(T&& value){
            return proxy_bool<T>(bool_flag::gthaneq, std::forward<T>(value), true);
        }

        template<typename T>
        proxy_bool<T> operator>=(T&& value){
            return proxy_bool<T>(bool_flag::lthaneq, std::forward<T>(value), true);
        }

        template<typename T>
        proxy_bool<T> operator&&(const proxy_bool<T>& other){
            return proxy_bool<T>{[=](const auto &arg) { return !arg && other.get_pred()(arg); }};
        }

        template<typename T>
        proxy_bool<T> operator||(const proxy_bool<T>& other){
            return proxy_bool<T>{[=](const auto &arg) { return !arg || other.get_pred()(arg); }};
        }
};

template<typename InT, typename OutT, typename Iterator>
class in_impl : public implicit_convertable<InT,OutT,Iterator>{
    public:
        using implicit_convertable<InT,OutT,Iterator>::implicit_convertable;

        template<auto P>
        if_impl<InT,OutT,Iterator> _if(pred<P>&&){
            PredFunctor<InT> predFunctor = P;
            return if_impl<InT,OutT,Iterator>(std::move(*this), std::move(predFunctor),pred_flag);
        }

        if_impl<InT,OutT,Iterator> _if(placeholder&) {
            PredFunctor<InT> predFunctor = [] (const auto& val) ->bool { return val; };
            return if_impl<InT,OutT,Iterator>(std::move(*this), std::move(predFunctor),pred_flag);
        }

        template<typename TT>
        if_impl<InT,OutT,Iterator> _if(proxy_bool<TT> &&proxy){
            PredFunctor<InT> pred = [&](auto arg) { return proxy.get_pred()(arg); };
            return if_impl<InT,OutT,Iterator>(std::move(*this), std::move(pred),pred_flag);
        }

        if_impl<InT,OutT,Iterator> _if(proxy_bool<InT> &&proxy){
            return if_impl<InT,OutT,Iterator>(std::move(*this), proxy.get_pred(),pred_flag);
        }

        if_impl<InT,OutT,Iterator> _if(not_proxy_bool&&){
            PredFunctor<InT> pred = [&](const auto &arg) ->bool { return !arg; };
            return if_impl<InT,OutT,Iterator>(std::move(*this), std::move(pred),pred_flag);
        }

        if_impl<InT,OutT,Iterator> _if(PredFunctor<InT> predF){
            return if_impl<InT, OutT, Iterator>(std::move(*this), std::move(predF), pred_flag);
        }
};

template<auto F>
class for_impl{
    public:
        for_impl() = default;

        template <template<typename> typename Cont, typename T>
        auto _in(const Cont<T> &container){
            static_assert(is_cont_v<Cont,T>, "argument to _in is not a container type");
            using OutT = typename function_ptr<decltype(F)>::ReturnType;
            TransFunctor<T, OutT> trans = F;
            using Iterator = decltype(container.begin());
            static_assert(std::is_same_v<decltype(container.begin()), decltype(container.end())>);
            return in_impl<T, OutT, Iterator>(container.begin(), container.end(), std::move(trans));
        }

        template <typename T>
        auto _in(std::initializer_list<T> &&container){
            using OutT = typename function_ptr<decltype(F)>::ReturnType;
            TransFunctor<T, OutT> trans = F;
            using Iterator = decltype(std::begin(container));
            static_assert(std::is_same_v<decltype(std::begin(container)), decltype(std::end(container))>);
            return in_impl<T, OutT, Iterator>(std::begin(container), std::end(container), std::move(trans));
        }

        template<typename T, size_t Size>
        auto _in(const T(&array)[Size]){
            using OutT = typename function_ptr<decltype(F)>::ReturnType;
            TransFunctor<T, OutT> trans = F;
            using Iterator = decltype(std::begin(array));
            static_assert(std::is_same_v<decltype(std::begin(array)), decltype(std::end(array))>);
            return in_impl<T, OutT, Iterator>(std::begin(array), std::end(array), std::move(trans));
        }
};

template<>
class for_impl<0>{
    public:
        for_impl() = default;

        template <template<typename> typename Cont, typename T>
        auto _in(const Cont<T> &container){
            static_assert(is_cont_v<Cont,T>, "argument to _in is not a container type");
            using Iterator = decltype(container.begin());
            static_assert(std::is_same_v<decltype(container.begin()), decltype(container.end())>);
            return in_impl<T, T, Iterator>(container.begin(), container.end());
        }

        template <typename T>
        auto _in(const std::initializer_list<T> &container){
            using Iterator = decltype(std::begin(container));
            static_assert(std::is_same_v<decltype(std::begin(container)), decltype(std::end(container))>);
            return in_impl<T,T,Iterator>(std::begin(container),std::end(container));
        }

        template<typename T, size_t Size>
        auto _in(const T(&array)[Size]){
            using Iterator = decltype(std::begin(array));
            static_assert(std::is_same_v<decltype(std::begin(array)), decltype(std::end(array))>);
            return in_impl<T, T, Iterator>(std::begin(array), std::end(array));
        }
};

template<typename T>
struct range_iter : public std::iterator<std::forward_iterator_tag,T> {
    T value;
    T jump;

    range_iter(T val, T j) : value{val}, jump{j} {};

    T operator*() { return value; }

    range_iter &operator++() { 
        value+=jump;
        return *this;
    }

    range_iter operator++(int) { 
        value+=jump;
        return range_iter{value - jump, jump};
    }

    range_iter &operator--() { 
        value-=jump;
        return *this;
    }

    range_iter operator--(int) { 
        value-=jump;
        return range_iter{value + jump, jump};
    }

    bool operator==(const range_iter& other) const {
        if(jump>0){
            return value >= other.value;
        }
        return value <= other.value;
    }

    bool operator!=(const range_iter& other) const {
        if(jump>0){
            return value < other.value;
        }
            return value > other.value;
    }
};

#ifdef LISTCOMP_CONVERTABLES
template<typename UT, template<typename...> typename... Ts>
struct range_impl_oper : public range_impl_oper<UT,Ts>... {

};

template<typename UT, template<typename...> typename T>
struct range_impl_oper<UT,T> {
    virtual range_iter<UT> begin() const = 0;
    virtual range_iter<UT> end() const = 0;

    ADD_LIST_COMP_OPERATOR(T, UT);
};

template<typename T>
struct _range : public range_impl_oper<T,LISTCOMP_CONVERTABLES> {
#else
template<typename T>
struct _range {
#endif
    T limit;
    T init;
    T jump;

    _range() = delete;
    _range(T val) : init{0}, limit{val}, jump{1} {};
    _range(T in, T end) : init{in}, limit{end}, jump{1} {};
    _range(T in, T end, T j) : init{in}, limit{end}, jump{j} {};

    range_iter<T> begin() const { return range_iter<T>{init, jump}; }
    range_iter<T> end() const { return range_iter<T>{limit, jump}; }
    range_iter<T> begin() { return range_iter<T>{init, jump}; }
    range_iter<T> end() { return range_iter<T>{limit, jump}; }

    ADD_LIST_COMP_OPERATOR(std::vector, T);

#ifndef LISTCOMP_DISABLE_STD_CONTAINERS
    ADD_LIST_COMP_OPERATOR(std::deque, T);
    ADD_LIST_COMP_OPERATOR(std::list, T);
    ADD_LIST_COMP_OPERATOR(std::forward_list, T);
#endif
};

} //namespace impl

class placeholder{
    private:
        const int id;

        inline static char inst_cnt = 0;

        placeholder(int i) : id{i} { inst_cnt++; }
        placeholder(placeholder&& other) : id{other.id} {};

        template <auto> friend class trans;

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

        template<typename T>
        impl::proxy_bool<T> operator==(T&& value){
            return impl::proxy_bool<T>(impl::bool_flag::equals, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_bool<T> operator!=(T&& value){
            return impl::proxy_bool<T>(impl::bool_flag::nequals, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_bool<T> operator<(T&& value){
            return impl::proxy_bool<T>(impl::bool_flag::lthan, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_bool<T> operator>(T&& value){
            return impl::proxy_bool<T>(impl::bool_flag::gthan, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_bool<T> operator<=(T&& value){
            return impl::proxy_bool<T>(impl::bool_flag::lthaneq, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_bool<T> operator>=(T&& value){
            return impl::proxy_bool<T>(impl::bool_flag::gthaneq, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_bool<T> operator==(T&& value, placeholder&){
            return impl::proxy_bool<T>(impl::bool_flag::requals, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_bool<T> operator!=(T&& value, placeholder&){
            return impl::proxy_bool<T>(impl::bool_flag::rnequals, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_bool<T> operator<(T&& value, placeholder&){
            return impl::proxy_bool<T>(impl::bool_flag::rlthan, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_bool<T> operator>(T&& value, placeholder&){
            return impl::proxy_bool<T>(impl::bool_flag::rgthan, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_bool<T> operator<=(T&& value, placeholder&){
            return impl::proxy_bool<T>(impl::bool_flag::rlthaneq, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_bool<T> operator>=(T&& value, placeholder&){
            return impl::proxy_bool<T>(impl::bool_flag::rgthaneq, std::forward<T>(value));
        }

        impl::not_proxy_bool operator!(){
            return impl::not_proxy_bool{};
        }

        template<typename T>
        impl::proxy_bool<T> operator&&(const impl::proxy_bool<T>& proxy){
            return impl::proxy_bool<T>{[&](const auto &arg) { return arg && proxy.get_pred()(arg); }};
        }

        template<typename T>
        impl::proxy_bool<T> operator||(const impl::proxy_bool<T>& proxy){
            return impl::proxy_bool<T>{[&](const auto &arg) { return arg || proxy.get_pred()(arg); }};
        }

        template<typename T>
        impl::proxy_trans<T> operator*(T&& value){
            return impl::proxy_trans<T>(impl::oper_flag::mult, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_trans<T> operator/(T&& value){
            return impl::proxy_trans<T>(impl::oper_flag::div, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_trans<T> operator+(T&& value){
            return impl::proxy_trans<T>(impl::oper_flag::add, std::forward<T>(value));
        }
        
        template<typename T>
        impl::proxy_trans<T> operator-(T&& value){
            return impl::proxy_trans<T>(impl::oper_flag::sub, std::forward<T>(value));
        }

        template<typename T>
        impl::proxy_trans<T> operator%(T&& value){
            return impl::proxy_trans<T>(impl::oper_flag::mod, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_trans<T> operator*(T&& value, placeholder&){
            return impl::proxy_trans<T>(impl::oper_flag::rmult, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_trans<T> operator/(T&& value, placeholder&){
            return impl::proxy_trans<T>(impl::oper_flag::rdiv, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_trans<T> operator+(T&& value, placeholder&){
            return impl::proxy_trans<T>(impl::oper_flag::radd, std::forward<T>(value));
        }
        
        template<typename T>
        friend impl::proxy_trans<T> operator-(T&& value, placeholder&){
            return impl::proxy_trans<T>(impl::oper_flag::rsub, std::forward<T>(value));
        }

        template<typename T>
        friend impl::proxy_trans<T> operator%(T&& value, placeholder&){
            return impl::proxy_trans<T>(impl::oper_flag::rmod, std::forward<T>(value));
        }

}_i,_j,_k;

template <auto F>
class trans {
    public:
        trans(placeholder &) {
            using FuncSpec = impl::function_ptr<decltype(F)>;
            static_assert(FuncSpec::value, "only function pointers can be passed as template arguments to trans");
            static_assert(FuncSpec::ArgSize == 1, "trans functions must take only one argument");
            static_assert(!std::is_same_v<typename FuncSpec::ArgType, void>, "trans functions must not take void-type arguments");
            static_assert(!std::is_same_v<typename FuncSpec::ReturnType, void>, "trans functions must not have void return-type");
        };
        trans() = delete;
        trans(trans &) = delete;
        trans(trans &&) = delete;
        trans &operator=(trans &) = delete;
        trans &operator=(trans &&) = delete;

        impl::for_impl<F> _for(placeholder &){
            return impl::for_impl<F>{};
        }
};

template <auto F>
using pred = impl::pred<F>;

template<typename T, typename=std::enable_if_t<std::is_arithmetic_v<T>>>
impl::_range<T> _range(T start, T end, T jump=1){
    return impl::_range<T>{start, end, jump};
}

template<typename T, typename=std::enable_if_t<std::is_arithmetic_v<T>>>
impl::_range<T> _range(T end){
    return impl::_range<T>{end};
}

} //namespace listcomp

#endif