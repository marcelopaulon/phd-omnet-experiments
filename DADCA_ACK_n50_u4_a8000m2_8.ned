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

network DADCA_ACK_n50_u4_a8000m2_8
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
        @display("i=misc/sensor;p=1572.55,7941.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2075.05,7812.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2745.20,7811.05");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3336.59,7980.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=40.66,7625.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1207.85,7535.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3627.85,7739.24");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4073.94,7625.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=848.75,7377.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1986.11,7345.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2465.46,7282.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2900.18,7260.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3264.99,7420.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5732.98,7303.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=26.50,7226.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=512.97,7249.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=787.78,7019.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1371.61,7217.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3162.12,7024.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3577.85,7104.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4269.07,7160.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=432.83,6858.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1621.96,6800.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3782.04,6769.77");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=55.10,6671.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1284.53,6609.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2106.01,6712.99");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2527.71,6749.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4200.85,6725.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5100.48,6663.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5702.46,6651.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=644.34,6455.28");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1754.29,6429.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2130.70,6303.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3171.90,6356.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3539.38,6428.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3996.92,6342.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4715.59,6317.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5323.71,6392.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5890.07,6320.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1018.75,6033.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2704.42,6142.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4338.23,6173.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=535.36,5999.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1733.21,5915.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3108.79,5957.86");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3668.22,5988.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4130.81,5846.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4922.75,5829.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5310.93,5996.13");
    }

}

