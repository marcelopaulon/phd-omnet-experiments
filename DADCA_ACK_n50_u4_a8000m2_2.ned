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

network DADCA_ACK_n50_u4_a8000m2_2
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
        @display("i=misc/sensor;p=190.58,7980.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1640.56,7927.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2710.87,7985.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3130.11,7802.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5400.65,7907.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5727.19,7755.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=518.10,7682.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=864.62,7562.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1346.78,7679.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1821.63,7563.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4036.01,7530.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4533.52,7659.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5100.05,7658.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=240.54,7455.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2333.01,7348.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3014.24,7331.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4928.90,7292.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=620.05,7107.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1074.47,7064.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1564.56,7068.64");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2582.87,7074.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3603.74,7116.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4269.58,7176.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4624.41,7089.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5173.10,7010.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5698.71,7208.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2131.84,6960.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3117.61,6915.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4252.35,6791.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=111.78,6660.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=549.45,6744.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1250.94,6726.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1735.55,6649.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3591.22,6682.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4753.96,6698.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5808.59,6609.64");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=291.20,6274.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=902.65,6425.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1405.46,6281.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2309.88,6470.02");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2685.44,6505.27");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3885.33,6473.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4700.35,6320.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5214.25,6457.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5983.38,6277.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2852.05,6096.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3501.40,6235.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4043.05,6139.36");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5080.20,6109.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5594.86,6225.15");
    }

}

