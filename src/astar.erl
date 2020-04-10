%% Route finding utility using A* algorithm

-module(astar).

-export([create_map/1, generate_grids/0, path/3]).

-type coord() :: {integer(), integer()}.

-record(map, {slab_height, slab_width, points = []}).
-record(score, {g = 0, h = 0, f = 0}).

%% creates the map with the specified grid structure
-spec create_map(list(tuple())) -> #map{}.
create_map(Grid) ->
    SlabHeight = 1,
    SlabWidth = 1,
    #map{slab_height = SlabHeight, slab_width = SlabWidth, points = Grid}.

-spec generate_grids() -> list(tuple()).
generate_grids() ->
    [
    {0,0,0,0,0},
    {0,0,1,1,0},
    {0,0,0,1,0},
    {0,0,0,1,0}
    ].

%% calculate distance between two points on the map
-spec distance(coord(), coord(), #map{}) -> float().
distance({FR, FC}, {TR, TC}, #map{slab_height = GridH, slab_width = GridW}) ->
    if FR == TR ->
        abs(TC - FC);
    FC == TC ->
        abs(TR - FR);
    FR - TR == FC - TC ->
        math:sqrt(math:pow(TR - FR, 2) + math:pow(TC - FC, 2));
    true ->
        DiagonalGrids = min(abs(TR - FR), abs(TC - FC)),
        {HeightGrids, LengthGrids} = if abs(TR - FR) > abs(TC - FC) ->
            {abs(abs(TR - FR) - abs(TC - FC)), 0};
        abs(TR - FR) =< abs(TC - FC) ->
            {0, abs(abs(TC - FC) - abs(TR - FR))}
        end,
        HeightGrids * GridH + DiagonalGrids * math:sqrt(math:pow(GridH, 2) + math:pow(GridW, 2)) + LengthGrids * GridW
    end.

%% Checks if movement on a grid is allowed or not
-spec grid_val(coord(), list(tuple())) -> false | 0 | 1.
grid_val({X, Y}, Grids) -> 
    RowPresent = X =< length(Grids) - 1, 
    if RowPresent ->
        ColPresent = Y =< size(hd(Grids)) - 1, 
        if ColPresent -> 
            element(Y+1, lists:nth(X+1, Grids)); 
        true -> false 
        end; 
    true -> false 
    end.

-spec find_neighbours(coord(), #map{}) -> list(coord()).
find_neighbours({X, Y}, Map) ->
    Points = [{X+1, Y}, {X-1, Y}, {X, Y+1}, {X, Y-1}, {X+1, Y+1}, {X+1, Y-1}, {X-1, Y-1}, {X-1, Y+1}],
    lists:filter(fun({Xx,Yy}) when Xx >=0 andalso Yy >= 0 -> grid_val({Xx, Yy}, Map#map.points) == 0; (_) -> false end, Points).

%% Tracking the visited path to the destination
-spec tracked_path(coord(), #{}) -> list(coord()).
tracked_path(Point, Closed) ->
    {Parent, _} = maps:get(Point, Closed),
    if Parent =/= Point ->
        [Point | tracked_path(Parent, Closed)];
    true ->
        [Point]
    end.

min(Key, {_Parent, #score{f = F}}, {undefined, undefined}) -> {Key, F};
min(Key, {_Parent, #score{f = F}}, {_KeyMinF, FMin}) when F < FMin -> {Key, F};
min(_Key, _Value, {MinKey, MinF}) -> {MinKey, MinF}.

%% Function to find shortest available path between two points on the map
-spec path(coord(), coord(), #map{}) -> incorrect_input | not_reachable | list(tuple()).
path(Start, End, #map{points = Grid} = Map) ->
    StartPoint = grid_val(Start, Grid),
    EndPoint = grid_val(End, Grid),
    if StartPoint =/= false andalso EndPoint =/= false ->
        Open = maps:put(Start, {Start, #score{}}, maps:new()),
        path(Start, End, Map, Open, maps:new());
    true -> incorrect_input
    end.

path(Start, End, Map, Open, Closed) ->
    {CurrNode, _} = maps:fold(fun min/3, {undefined, undefined}, Open),
    if CurrNode == undefined ->
        not_reachable;
    true ->
        {CurrNodeInfo, OpenMod} = maps:take(CurrNode, Open),
        ClosedMod = maps:put(CurrNode, CurrNodeInfo, Closed),
        {_CurrParent, #score{g = CurrG}} = CurrNodeInfo,
        if CurrNode == End ->
            tracked_path(End, ClosedMod);
        true ->
            Neighbours = find_neighbours(CurrNode, Map),
            AccOpenF = lists:foldl(fun(Neighbour, OpenAcc) ->
                IsPresent = maps:is_key(Neighbour, ClosedMod),
                if IsPresent -> OpenAcc;
                true ->
                    G = CurrG + distance(Neighbour, CurrNode, Map),
                    H = distance(Neighbour, End, Map),
                    F = G + H,
                    InsideOpen = maps:is_key(Neighbour, OpenAcc),
                    if InsideOpen ->
                        {_NP, #score{g = NG}} = maps:get(Neighbour, OpenAcc),
                        if G > NG -> OpenAcc;
                        true -> maps:put(Neighbour, {CurrNode, #score{g = G, h = H, f = F}}, OpenAcc)
                        end;
                    true -> maps:put(Neighbour, {CurrNode, #score{g = G, h = H, f = F}}, OpenAcc)
                    end
                end
            end, OpenMod, Neighbours),
            path(Start, End, Map, AccOpenF, ClosedMod)
        end
    end.
