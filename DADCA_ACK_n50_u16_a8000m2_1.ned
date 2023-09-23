package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_u16_a8000m2_1
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=187.59,7829.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=786.53,7953.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2142.48,7901.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2768.60,7929.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3510.76,7886.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3912.09,7900.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4977.95,7938.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5618.76,7969.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1722.12,7752.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2308.99,7591.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3666.46,7552.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4533.59,7656.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5279.92,7745.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=130.87,7296.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=708.59,7371.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1413.14,7453.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1725.87,7291.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2595.28,7328.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3080.98,7473.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4187.64,7489.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5104.38,7431.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5557.23,7466.18");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2246.17,7046.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3497.47,7199.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4041.40,7125.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4499.21,7230.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4988.17,7071.10");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=350.61,6894.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1394.87,6776.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1913.25,6850.88");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2917.41,6925.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5567.01,6788.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=836.55,6683.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1737.92,6521.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2343.30,6695.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2708.14,6624.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3121.02,6578.13");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3518.18,6622.24");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4119.72,6712.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4497.59,6607.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4911.71,6700.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=313.10,6326.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4204.02,6302.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5542.31,6429.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=572.96,6089.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=975.72,6219.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1715.29,6127.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2568.05,6129.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3264.38,6177.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3658.79,6216.88");
    }

}

