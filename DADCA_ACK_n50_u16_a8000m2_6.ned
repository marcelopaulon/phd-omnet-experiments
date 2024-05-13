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

network DADCA_ACK_n50_u16_a8000m2_6
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
        @display("i=misc/sensor;p=328.42,7784.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=757.85,7763.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1711.51,7939.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2274.31,7925.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2683.26,7844.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3851.01,7828.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5742.93,7994.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1142.97,7577.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3335.48,7629.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4734.55,7559.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5163.18,7611.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5600.04,7586.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=38.52,7350.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=569.72,7445.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1859.45,7372.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2452.82,7481.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2881.55,7278.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3574.20,7298.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4171.62,7504.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5828.14,7313.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1031.96,7066.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1539.69,7015.59");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3841.97,7035.87");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4506.01,7130.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5157.46,7165.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=225.45,6957.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1980.50,6848.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2406.02,6928.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2755.45,6798.77");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3383.87,6890.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4689.66,6774.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5582.61,6834.76");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=819.29,6533.64");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3733.47,6541.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4055.60,6757.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=467.05,6338.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1218.40,6408.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2282.73,6346.48");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3177.34,6355.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5054.79,6382.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5462.10,6301.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=83.46,6195.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=953.10,6101.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1326.38,6047.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1803.53,6208.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3489.96,6047.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4112.87,6071.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4516.28,6184.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5913.68,6229.40");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=188.85,5818.54");
    }

}

