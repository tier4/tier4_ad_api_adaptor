# autoware_failure_notification

## Condition

### LocalizationState

APIの`/api/localization/initialization_state`が指定した値のいずれかになった場合にTrueとなります。指定できる値は以下の通りです。ステートが未受信の場合はUnknownとして扱われます。

- Unknwon
- Uninitialized
- Initializing
- Initialized

使用例

```txt
LocalizationState(Uninitialized, Initializing)
LocalizationState(Initialized)
```

### RouteState

APIの`/api/routing/state`が指定した値のいずれかになった場合にTrueとなります。指定できる値は以下の通りです。ステートが未受信の場合はUnknownとして扱われます。

- Unknown
- Unset
- Set
- Arrived

使用例

```txt
RouteState(Unset)
RouteState(Set, Arrived)
```

### Not

指定した式の真偽を反転させます。引数にはconditionとして解釈できる単一の式を指定してください。

使用例

```txt
Not(LocalizationState(Initialized))
Not(RouteState(Set, Arrived))
```
