#pragma once
#include "core.hh"


//
// Entities
//

using Entity = entt::entity;
inline constexpr auto nullEntity = entt::null;


// The global entity registry
inline entt::registry registry;

// Create a new entity
inline Entity createEntity() {
  return registry.create();
}
inline Entity createEntity(Entity hint) {
  return registry.create(hint);
}

// Destroy an entity
inline Seq<void (*)(Entity), 32> componentRemovers;
inline void destroyEntity(Entity ent) {
  for (auto &remover : componentRemovers) {
    remover(ent);
  }
  registry.destroy(ent);
}

// Check that an entity exists
inline bool exists(Entity ent) {
  return registry.valid(ent);
}


//
// Components
//

// Component pools
template<typename T>
inline void remove(Entity ent);
template<typename F>
void each(F &&f);
inline bool entitiesClearedOnExit = false;
template<typename T>
struct ComponentPool : entt::storage<T> {
  ComponentPool() {
    componentRemovers.emplace_back(remove<T>);
  }
  ~ComponentPool() {
    if (!entitiesClearedOnExit) {
      each(destroyEntity);
      entitiesClearedOnExit = true;
    }
  }
};
template<typename T>
inline ComponentPool<T> componentPool;

// Check whether entity has a component
template<typename T>
inline bool has(Entity ent) {
  return componentPool<T>.contains(ent);
}

// Get a component on an entity
template<typename T>
inline decltype(auto) get(Entity ent) {
  return componentPool<T>.get(ent);
}

// Add a component to an entity
template<typename T>
inline decltype(auto) add(Entity ent, T &&value = {}) {
  if (has<T>(ent)) {
    return get<T>(ent);
  } else {
    auto &comp = componentPool<T>.emplace(ent, std::forward<T>(value));
    if constexpr (requires { add(comp, ent); }) {
      add(comp, ent);
    }
    return comp;
  }
}

// Remove a component from an entity
template<typename T>
void remove(Entity ent) {
  if (has<T>(ent)) {
    if constexpr (requires { remove(*((T *)nullptr), ent); }) {
      remove(get<T>(ent), ent);
    }
    componentPool<T>.remove(ent);
  }
}

// Query component combinations
template<typename T>
struct Each {};
template<typename R, typename C>
struct Each<R (C::*)(Entity ent) const> {
  template<typename F>
  static void each(F &&f) {
    registry.each(std::forward<F>(f));
  }
};
template<typename R, typename C, typename T, typename... Ts>
struct Each<R (C::*)(Entity ent, T &, Ts &...) const> {
  static void each(auto &&f) {
    for (auto ent : static_cast<entt::sparse_set &>(componentPool<T>)) {
      if ((has<Ts>(ent) && ...)) {
        f(ent, get<T>(ent), get<Ts>(ent)...);
      }
    }
  }
};
template<typename R, typename... Args>
inline void each(R (&f)(Args...)) {
  each([&](Args... args) {
    f(std::forward<Args>(args)...);
  });
}
template<typename F>
inline void each(F &&f) {
  if constexpr (requires { &F::operator(); }) {
    Each<decltype(&F::operator())>::each(std::forward<F>(f));
  } else {
    each(std::forward<F>(f));
  }
}

// Sort component data
template<typename T, typename F>
inline void sort(F &&f) {
  componentPool<T>.sort(std::forward<F>(f));
}
