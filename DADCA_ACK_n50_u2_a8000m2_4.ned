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

network DADCA_ACK_n50_u2_a8000m2_4
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
        @display("i=misc/sensor;p=225.35,7928.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=700.12,7923.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1181.02,7776.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1975.06,7879.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2306.84,7760.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3479.35,7763.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3805.92,7973.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4714.37,7871.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5137.91,7942.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5622.41,7904.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1507.77,7626.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2827.05,7746.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4299.97,7662.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4825.60,7509.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=323.39,7423.76");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=933.44,7386.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2530.30,7477.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3173.54,7423.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3625.30,7411.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4053.02,7400.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4464.59,7293.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5724.47,7291.83");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=401.47,7063.56");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1321.02,7204.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1954.89,7188.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2343.06,7035.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3049.89,7028.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=901.08,6769.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1282.91,6816.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2003.54,6780.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2699.04,6832.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3523.47,6950.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4065.97,6822.48");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4679.25,6989.06");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5636.71,6848.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=58.44,6571.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=457.21,6551.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1537.45,6542.46");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4413.81,6649.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3196.57,6358.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3704.28,6310.42");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4921.62,6410.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5400.61,6493.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=71.19,6123.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=428.63,6109.59");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1111.38,6262.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2099.48,6226.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2500.63,6160.50");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2935.92,6123.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4200.30,6241.21");
    }

}

