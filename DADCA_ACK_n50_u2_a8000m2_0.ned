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

network DADCA_ACK_n50_u2_a8000m2_0
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
        @display("i=misc/sensor;p=237.03,7919.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=662.89,7852.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1877.28,7914.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3698.73,7944.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4414.05,7877.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4976.28,7810.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1415.68,7612.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2290.33,7659.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2721.39,7740.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3241.59,7582.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3892.15,7532.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5452.97,7676.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=109.14,7482.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=754.15,7478.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1124.93,7385.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4717.57,7364.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5797.73,7275.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=563.46,7073.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1801.64,7217.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2615.57,7074.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3465.14,7089.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4138.07,7197.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5158.82,7064.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=179.24,6977.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1465.86,6924.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1806.40,6830.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3889.85,6783.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4664.80,6797.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=834.44,6753.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1217.75,6532.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2139.23,6611.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2608.73,6612.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3098.58,6522.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5241.82,6558.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=355.68,6340.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=878.00,6346.33");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3427.79,6400.49");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4176.31,6467.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4956.19,6304.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5870.82,6484.08");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1222.36,6115.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1735.65,6204.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2777.08,6113.76");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4601.34,6164.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=89.32,5879.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=496.08,5987.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=873.76,5971.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1920.45,5816.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2291.25,6004.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3293.32,5959.69");
    }

}

