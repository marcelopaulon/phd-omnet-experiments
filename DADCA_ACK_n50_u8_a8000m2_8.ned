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

network DADCA_ACK_n50_u8_a8000m2_8
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
        @display("i=misc/sensor;p=621.91,7868.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1351.91,7851.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2161.51,7969.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2981.24,7859.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4179.04,7902.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5187.95,7789.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=514.01,7533.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1173.28,7535.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2446.64,7688.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3586.92,7707.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3998.05,7549.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4645.80,7738.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1828.00,7373.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2808.00,7437.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4484.67,7392.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5403.92,7305.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=159.15,7078.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1071.57,7052.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1468.72,7175.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2590.28,7021.53");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2954.77,7115.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3356.23,7098.53");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4283.41,7022.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4961.07,7131.37");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=636.65,6990.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2178.52,6811.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3872.70,6994.33");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5417.60,6909.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5799.70,6894.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=598.30,6576.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1270.15,6674.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3523.21,6724.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4013.51,6611.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4770.67,6754.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5123.44,6649.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=66.47,6489.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1956.12,6310.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2588.57,6409.68");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3043.25,6505.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3689.57,6396.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4481.78,6395.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5260.88,6303.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5723.74,6484.48");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=240.66,6083.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=846.88,6159.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1250.00,6027.13");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2254.66,6049.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2860.03,6155.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3328.92,6082.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4083.98,6036.26");
    }

}

