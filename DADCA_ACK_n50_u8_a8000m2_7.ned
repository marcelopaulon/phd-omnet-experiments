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

network DADCA_ACK_n50_u8_a8000m2_7
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
        @display("i=misc/sensor;p=554.71,7812.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1019.97,7909.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1495.94,7849.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2159.24,7853.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2717.01,7937.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3416.07,7950.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4182.47,7818.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5166.47,7775.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5480.37,7967.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=165.87,7632.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=800.46,7534.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1841.48,7558.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2491.58,7581.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3099.00,7689.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3700.20,7657.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4563.84,7534.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5457.82,7516.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5890.81,7342.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=717.60,7122.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1316.04,7064.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1660.37,7222.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2955.94,7164.10");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3685.94,7220.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5013.37,7017.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=228.00,6975.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2452.80,6970.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5935.47,6831.32");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=940.13,6541.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1688.86,6664.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2096.42,6591.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3190.93,6702.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4385.02,6747.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5165.48,6673.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=400.37,6366.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1543.88,6297.27");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2378.44,6301.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2803.82,6472.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3354.96,6323.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3808.49,6478.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4848.29,6420.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5578.05,6473.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5976.90,6287.77");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=865.70,6073.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4083.61,6086.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4457.09,6220.80");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=165.81,5940.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1873.71,5981.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3220.61,5993.27");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3714.69,6014.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5255.78,5986.76");
    }

}

