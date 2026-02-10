# ArxTypeTraits

C++ type_traits for Arduino which cannot use it as default


## Features

- automatically use standard library first if the boards can
- if standard library is not enough (e.g. only C++11 is available), add missing parts listed below
- works almost all Arduino platforms (Let me know if you have errors)
- compatible with [ArduinoSTL](https://github.com/mike-matera/ArduinoSTL) or other [uClibc++](https://www.uclibc.org/) libraries
  - thx @matthijskooijman


## Supported Class Templates

### C++11 (defined only for boards before C++11)

- `std::integral_constant`
- `std::true_type`
- `std::false_type`
- `std::declval`
- `std::enable_if`
- `std::conditional`
- `std::remove_cv`
- `std::remove_const`
- `std::remove_volatile`
- `std::remove_pointer`
- `std::remove_reference`
- `std::remove_extent`
- `std::remove_all_extents`
- `std::add_cv`
- `std::add_const`
- `std::add_volatile`
- `std::add_pointer`
- `std::add_lvalue_reference`
- `std::add_rvalue_reference`
- `std::forward`
- `std::is_same`
- `std::is_integral`
- `std::is_floating_point`
- `std::is_arithmetic`
- `std::is_signed`
- `std::is_unsigned`
- `std::is_const`
- `std::is_volatile`
- `std::is_reference`
- `std::is_lvalue_reference`
- `std::is_rvalue_reference`
- `std::is_pointer`
- `std::is_member_pointer`
- `std::is_array`
- `std::is_convertible`
- `std::is_function`
- `std::is_empty`
- `std::is_void`
- `std::is_class`
- `std::is_scalar`
- `std::is_object`
- `std::is_base_of`
- `std::decay`
- `std::result_of`
- `std::rank`
- `std::extent`
- `std::addressof`


#### for utility

- `std::numeric_limits` (only `max()` and `min()` now)
- `std::swap`
- `std::initializer_list`
- `std::tuple`
- `std::get`
- `std::tuple_size`
- `std::function`
- `std::reference_wrapper`
- `std::ref`
- `std::cref`
- `std::as_const`


### C++14 (defined only for boards before C++14)

- `std::enable_if_t`
- `std::decay_t`
- `std::remove_cv_t`
- `std::remove_const_t`
- `std::remove_volatile_t`
- `std::remove_reference_t`
- `std::remove_pointer_t`
- `std::remove_extent_t`
- `std::remove_all_extents_t`
- `std::add_cv_t`
- `std::add_const_t`
- `std::add_volatile_t`
- `std::add_pointer_t`
- `std::add_lvalue_reference_t`
- `std::add_rvalue_reference_t`
- `std::integer_sequence`
- `std::index_sequence`
- `std::make_index_sequence`
- `std::index_sequence_for`
- `std::is_null_pointer`


### C++17 (defined only for boards before C++17)

- `std::bool_constant`
- `std::is_same_v`
- `std::is_void_v`
- `std::is_class_v`
- `std::is_arithmetic_v`
- `std::is_const_v`
- `std::is_volatile_v`
- `std::is_reference_v`
- `std::is_lvalue_reference_v`
- `std::is_rvalue_reference_v`
- `std::is_pointer_v`
- `std::is_member_pointer_v`
- `std::is_null_pointer_v`
- `std::is_scalar_v`
- `std::is_array_v`
- `std::is_object_v`
- `std::is_function_v`
- `std::is_base_of_v`
- `std::rank_v`
- `std::extent_v`
- `std::void_t`
- `std::disjunction`
- `std::conjunction`
- `std::negation`
- `std::apply`
- `std::invoke_result`
- `std::invoke_result_t`


### C++2a

- `std::type_identity`
- `std::type_identity_t`
- `std::remove_cvref`
- `std::remove_cvref_t`
- `std::is_bounded_array`
- `std::is_bounded_array_v`
- `std::is_unbounded_array`
- `std::is_unbounded_array_v`


### Others (defined for all boards)

- `arx::is_detected`
- `arx::is_callable`
- `arx::function_traits`

(traits with the _v suffix are defined only when compiled as C++17 or higher)

## Used Inside of

- [Packetizer](https://github.com/hideakitai/Packetizer)
- [MsgPack](https://github.com/hideakitai/MsgPack)
- [MsgPacketizer](https://github.com/hideakitai/MsgPacketizer)
- [ArduinoOSC](https://github.com/hideakitai/ArduinoOSC)
- [ArtNet](https://github.com/hideakitai/ArtNet)
- [PollingTimer](https://github.com/hideakitai/PollingTimer)
- [Tween](https://github.com/hideakitai/Tween)
- [ArxStringUtils](https://github.com/hideakitai/ArxStringUtils)
- [Filters](https://github.com/hideakitai/Filters)
- [Debouncer](https://github.com/hideakitai/Debouncer)
- [ArduinoEigen](https://github.com/hideakitai/ArduinoEigen)


## Contributors

- [Matthijs Kooijman](https://github.com/matthijskooijman)


## License

MIT
