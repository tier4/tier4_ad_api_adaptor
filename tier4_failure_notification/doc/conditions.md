# Conditions

## True

常にTrueとなります。

使用例

```txt
True
```

## False

常にFalseとなります。

使用例

```txt
False
```

## Not

指定した式の真偽を反転させます。引数にはconditionとして解釈できる単一の式を指定してください。

使用例

```txt
Not(LocalizationState(Initialized))
Not(RouteState(Set, Arrived))
```

## And

指定した式の論理積を計算します。引数にはconditionとして解釈できる一つ以上の式を指定してください。

使用例

```txt
And(LocalizationState(Initialized), RouteState(Set))
And(True, True, True)
And(True, True, True, True)
```

## Or

指定した式の論理和を計算します。引数にはconditionとして解釈できる一つ以上の式を指定してください。

使用例

```txt
Or(LocalizationState(Initialized), RouteState(Set))
Or(True, True, True)
Or(True, True, True, True)
```

## LocalizationState

APIの`/api/localization/initialization_state`が指定した値のいずれかになった場合にTrueとなります。指定できる値は以下の通りです。ステートが未受信の場合はUnknownとして扱われます。

- Unknown
- Uninitialized
- Initializing
- Initialized

使用例

```txt
LocalizationState(Uninitialized, Initializing)
LocalizationState(Initialized)
```

## RouteState

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
