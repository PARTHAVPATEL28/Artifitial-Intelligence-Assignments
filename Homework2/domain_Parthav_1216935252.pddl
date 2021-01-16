(define (domain bookWorld)

    (:requirements
        :equality
        :typing
        :strips
    )

    (:types
        robot
        book
        bin
        location
        subject
        size
    )


    (:predicates
        (Book_At ?a - book ?b - location)
        (Bin_At ?a - bin ?b - location)
        (Book_Subject ?a - book ?b - subject)
        (Book_Size ?a - book ?b - size)
        (Bin_Subject ?a - bin ?b - subject)
        (Bin_Size ?a - bin ?b - size)
        (Robot_At ?a - robot ?b - location)
        (Empty_Basket ?a - robot)
        (In_Basket ?b - book)
    )

    (:action pick
        :parameters (?bk - book
                     ?loc - location
                     ?r - robot)
        :precondition (
            and (Robot_At ?r ?loc)(Empty_Basket ?r)(Book_At ?bk ?loc)
        )
        :effect (
            and (not (Empty_Basket ?r))(not (Book_At ?bk ?loc))(In_Basket ?bk)
        )
    )

    (:action place
        :parameters (?bk - book
                     ?sub - subject
                     ?bn - bin
                     ?sz - size
                     ?loc - location
                     ?r - robot)
        :precondition (
            and (Robot_At ?r ?loc)(Bin_At ?bn ?loc)(not (Empty_Basket ?r))(In_Basket ?bk)(Book_Subject ?bk ?sub)(Book_Size ?bk ?sz)(Bin_Subject ?bn ?sub)(Bin_Size ?bn ?sz)
        )
        :effect (
            and (not (In_Basket ?bk))(Book_At ?bk ?loc)(Empty_Basket ?r)
        )
    )

    (:action move
        :parameters (?r - robot
                     ?loc-from - location
                     ?loc-to - location)
        :precondition (
            Robot_At ?r ?loc-from
        )
        :effect (
            and (Robot_At ?r ?loc-to)(not (Robot_At ?r ?loc-from))
        )
    )

)