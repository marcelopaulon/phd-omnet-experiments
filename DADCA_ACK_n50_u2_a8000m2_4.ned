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
        @display("i=misc/sensor;p=301.20,7753.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3092.90,7989.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5100.60,7762.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=695.20,7593.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1883.70,7708.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2448.26,7730.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3501.61,7648.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4069.05,7693.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4691.39,7592.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=29.97,7291.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=541.05,7261.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1043.31,7287.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1936.07,7272.83");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2723.00,7444.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3074.56,7410.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5224.48,7365.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5621.21,7498.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5986.55,7424.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3653.04,7161.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4289.55,7076.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4843.97,7231.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5550.19,7081.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=591.02,6812.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1569.45,6878.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2458.30,6866.57");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2805.52,6777.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3130.28,6928.30");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4868.01,6801.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5201.57,6943.53");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5890.11,6969.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1076.98,6649.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1929.83,6755.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5263.63,6550.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=70.73,6432.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1940.56,6358.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2534.32,6484.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2961.44,6447.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3491.99,6505.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5910.73,6408.73");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=616.93,6245.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3217.26,6165.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4358.13,6214.52");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4931.61,6230.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5468.74,6073.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=129.92,5809.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=808.53,5873.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2343.74,5952.16");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2812.17,5950.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3520.97,5901.94");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3962.70,5910.94");
    }

}

