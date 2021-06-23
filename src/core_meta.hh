#pragma once
#include "core.hh"


//
// Hashing
//

using HashedString = entt::hashed_string;
using namespace entt::literals;

constexpr uint32_t hash(const char *str) {
  return entt::hashed_string(str).value();
}

constexpr uint32_t hash(std::string_view str) {
  return entt::hashed_string::value(str.data(), str.size());
}


//
// Type names
//

template<typename T>
constexpr std::string_view getTypeName() {
  constexpr auto prefixLength = 36, suffixLength = 1;
  const char *data = __PRETTY_FUNCTION__;
  const char *end = data;
  while (*end) {
    ++end;
  }
  return { data + prefixLength, size_t(end - data - prefixLength - suffixLength) };
}


//
// Component types list
//

template<int N>
struct ComponentTypeCounter : ComponentTypeCounter<N - 1> {
  static constexpr auto num = N;
};
template<>
struct ComponentTypeCounter<0> {
  static constexpr auto num = 0;
};
ComponentTypeCounter<0> numComponentTypes(ComponentTypeCounter<0>);

template<int I>
struct ComponentTypeList;
template<>
struct ComponentTypeList<0> {
  static void each(auto &&f) {
  }
};

static constexpr auto maxNumComponentTypes = 32;

#define ComponentTypeListAdd(T)                                                                    \
  static constexpr auto ComponentTypeList_##T##_Size                                               \
      = decltype(numComponentTypes(ComponentTypeCounter<maxNumComponentTypes>()))::num + 1;        \
  static_assert(ComponentTypeList_##T##_Size < maxNumComponentTypes);                              \
  ComponentTypeCounter<ComponentTypeList_##T##_Size> numComponentTypes(                            \
      ComponentTypeCounter<ComponentTypeList_##T##_Size>);                                         \
  template<>                                                                                       \
  struct ComponentTypeList<ComponentTypeList_##T##_Size> {                                         \
    static void each(auto &&f) {                                                                   \
      ComponentTypeList<ComponentTypeList_##T##_Size - 1>::each(f);                                \
      f((T *)nullptr);                                                                             \
    }                                                                                              \
  }

#define Comp(T)                                                                                    \
  T;                                                                                               \
  ComponentTypeListAdd(T);                                                                         \
  struct T

#define UseComponentTypes()                                                                        \
  void forEachComponentType(auto &&f) {                                                            \
    ComponentTypeList<decltype(numComponentTypes(                                                  \
        ComponentTypeCounter<maxNumComponentTypes>()))::num>::each(std::forward<decltype(f)>(f));  \
  }


//
// Utilitites based on standard library constructs
//

// RemoveReference
template<typename T>
struct RemoveReferenceStruct {
  using Type = T;
};
template<typename T>
struct RemoveReferenceStruct<T &> {
  using Type = T;
};
template<typename T>
struct RemoveReferenceStruct<T &&> {
  using Type = T;
};
template<typename T>
using RemoveReference = typename RemoveReferenceStruct<T>::Type;

// RemoveCV
template<typename T>
struct RemoveCVStruct {
  using Type = T;
};
template<typename T>
struct RemoveCVStruct<const T> {
  using Type = T;
};
template<typename T>
struct RemoveCVStruct<volatile T> {
  using Type = T;
};
template<typename T>
struct RemoveCVStruct<const volatile T> {
  using Type = T;
};
template<typename T>
using RemoveCV = typename RemoveCVStruct<T>::Type;

// IndexSequence
template<int... Index>
struct IndexSequence {
  static constexpr int size() noexcept {
    return sizeof...(Index);
  }
};

// Void
template<typename...>
using Void = void;

// declval
template<typename T, typename U = T &&>
U declval(int);
template<typename T>
T declval(long);
template<typename T>
auto declval() -> decltype(declval<T>(0)) {
  return declval<T>(0);
}

// isCharArray
template<typename T>
inline constexpr auto isCharArray = false;
template<unsigned N>
inline constexpr auto isCharArray<char[N]> = true;
template<unsigned N>
inline constexpr auto isCharArray<char (&)[N]> = true;


//
// Props
//

struct EditInspectAttribs {
  bool breakBefore = false;
};

struct PropAttribs {
  std::string_view name;
  EditInspectAttribs inspect;
};

template<typename Value, typename Internal>
struct PropHolder {
  Value value {};

  Value &operator()() {
    return value;
  }

  const Value &operator()() const {
    return value;
  }

  static constexpr std::string_view name = Internal::name;
  static constexpr uint32_t nameHash = hash(name);
  inline static constexpr const PropAttribs &attribs = Internal::attribs;
};

#define Prop(type, name_, ...) PropNamed(#name_, type, name_, __VA_ARGS__)
#define PropNamed(nameStr, type, name_, ...)                                                       \
  struct INTERNAL_##name_ {                                                                        \
    inline static constexpr std::string_view name = nameStr;                                       \
    inline static constexpr PropAttribs attribs { .name = name, __VA_ARGS__ };                     \
  };                                                                                               \
  PropHolder<PROP_PARENS_1(PROP_PARENS_3 type), INTERNAL_##name_> name_
#define PROP_PARENS_1(...) PROP_PARENS_2(__VA_ARGS__)
#define PROP_PARENS_2(...) NO##__VA_ARGS__
#define PROP_PARENS_3(...) PROP_PARENS_3 __VA_ARGS__
#define NOPROP_PARENS_3

struct CanConvertToAnything {
  template<typename Type>
  operator Type() const; // NOLINT(google-explicit-constructor)
};

template<typename Aggregate, typename Base = IndexSequence<>, typename = void>
struct CountReflectbleFields : Base {};

template<typename Aggregate, int... Indices>
struct CountReflectbleFields<Aggregate, IndexSequence<Indices...>,
    Void<decltype(Aggregate { { (static_cast<void>(Indices), declval<CanConvertToAnything>()) }...,
        { declval<CanConvertToAnything>() } })>>
    : CountReflectbleFields<Aggregate, IndexSequence<Indices..., sizeof...(Indices)>> {};

template<typename T>
constexpr int countReflectableFields() {
  return CountReflectbleFields<RemoveCV<RemoveReference<T>>>().size();
}

template<typename T>
inline constexpr auto isProp = false;
template<typename... Params>
inline constexpr auto isProp<PropHolder<Params...>> = true;
template<typename... Params>
inline constexpr auto isProp<PropHolder<Params...> &> = true;
template<typename... Params>
inline constexpr auto isProp<const PropHolder<Params...> &> = true;

template<typename T, typename F>
inline void forEachProp(T &val, F &&func) {
  const auto C = [&](auto &field) {
    if constexpr (isProp<decltype(field)>) {
      func(field);
    }
  };
  constexpr auto n = countReflectableFields<T>();
  static_assert(n <= 24, "forEachProp: only up to 24 fields supported");
  if constexpr (n == 1) {
    auto &[a] = val;
    (C(a));
  } else if constexpr (n == 2) {
    auto &[a, b] = val;
    (C(a), C(b));
  } else if constexpr (n == 3) {
    auto &[a, b, c] = val;
    (C(a), C(b), C(c));
  } else if constexpr (n == 4) {
    auto &[a, b, c, d] = val;
    (C(a), C(b), C(c), C(d));
  } else if constexpr (n == 5) {
    auto &[a, b, c, d, e] = val;
    (C(a), C(b), C(c), C(d), C(e));
  } else if constexpr (n == 6) {
    auto &[a, b, c, d, e, f] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f));
  } else if constexpr (n == 7) {
    auto &[a, b, c, d, e, f, g] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g));
  } else if constexpr (n == 8) {
    auto &[a, b, c, d, e, f, g, h] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h));
  } else if constexpr (n == 9) {
    auto &[a, b, c, d, e, f, g, h, i] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i));
  } else if constexpr (n == 10) {
    auto &[a, b, c, d, e, f, g, h, i, j] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j));
  } else if constexpr (n == 11) {
    auto &[a, b, c, d, e, f, g, h, i, j, k] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k));
  } else if constexpr (n == 12) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l));
  } else if constexpr (n == 13) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m));
  } else if constexpr (n == 14) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n));
  } else if constexpr (n == 15) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o));
  } else if constexpr (n == 16) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o),
        C(p));
  } else if constexpr (n == 17) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q));
  } else if constexpr (n == 18) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q), C(r));
  } else if constexpr (n == 19) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q), C(r), C(s));
  } else if constexpr (n == 20) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q), C(r), C(s), C(t));
  } else if constexpr (n == 21) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q), C(r), C(s), C(t), C(u));
  } else if constexpr (n == 22) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q), C(r), C(s), C(t), C(u), C(v));
  } else if constexpr (n == 23) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q), C(r), C(s), C(t), C(u), C(v), C(w));
  } else if constexpr (n == 24) {
    auto &[a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x] = val;
    (C(a), C(b), C(c), C(d), C(e), C(f), C(g), C(h), C(i), C(j), C(k), C(l), C(m), C(n), C(o), C(p),
        C(q), C(r), C(s), C(t), C(u), C(v), C(w), C(x));
  }
}

inline int getNumProps(auto &val) {
  auto result = 0;
  forEachProp(val, [&](auto &prop) {
    ++result;
  });
  return result;
}
