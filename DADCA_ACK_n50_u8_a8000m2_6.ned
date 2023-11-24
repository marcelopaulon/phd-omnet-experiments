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

network DADCA_ACK_n50_u8_a8000m2_6
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
        @display("i=misc/sensor;p=1116.45,7953.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1573.89,7878.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2004.67,7926.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2945.60,7776.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4941.06,7880.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5402.99,7908.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5853.63,7967.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=889.87,7643.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1327.81,7603.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1950.85,7571.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2524.52,7595.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4139.90,7547.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4799.34,7521.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1123.28,7274.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3552.51,7465.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5184.58,7409.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=505.03,7193.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1804.75,7196.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2273.41,7102.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2683.61,7231.72");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2976.08,7015.37");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3697.80,7131.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4203.67,7128.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4579.14,7240.50");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=104.62,6827.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1390.34,6890.49");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2611.83,6836.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3291.90,6841.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4725.94,6821.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5385.07,6911.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=812.42,6678.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1980.60,6576.66");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3600.57,6616.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4108.18,6736.03");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1510.68,6348.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2805.91,6492.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3178.71,6346.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4219.84,6401.64");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5138.40,6454.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5815.15,6439.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=344.75,6121.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1097.63,6259.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1828.80,6160.60");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2481.17,6237.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2776.84,6046.41");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3388.27,6058.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4501.51,6176.33");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4856.06,6220.73");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5398.28,6110.88");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=702.30,6007.78");
    }

}

