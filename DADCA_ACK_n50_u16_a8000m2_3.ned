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

network DADCA_ACK_n50_u16_a8000m2_3
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
        @display("i=misc/sensor;p=791.56,7994.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1245.26,7932.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1561.56,7765.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2417.44,7943.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2941.66,7761.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3949.69,7810.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4872.96,7799.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5543.38,7823.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=435.01,7731.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1069.61,7564.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3373.43,7737.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4317.59,7685.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=75.70,7428.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1670.64,7381.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2152.94,7482.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2545.14,7359.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3152.90,7450.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3983.95,7264.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4667.68,7323.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5370.63,7435.82");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5831.10,7305.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=605.70,7224.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2853.48,7131.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3425.05,7215.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5097.27,7036.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=301.75,6949.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=739.56,6774.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1501.23,7008.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2089.68,6952.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2556.55,6823.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3117.14,6788.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3816.53,6769.27");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4333.84,6812.01");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5506.90,6946.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1256.92,6703.97");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4950.84,6708.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=445.23,6308.40");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=928.51,6329.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1598.09,6496.28");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2193.18,6387.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2547.62,6354.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3111.73,6350.93");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3676.16,6267.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4373.26,6362.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5286.37,6488.03");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1336.53,6087.67");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1811.46,6054.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4897.24,6066.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=76.30,5905.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=554.02,5868.92");
    }

}

