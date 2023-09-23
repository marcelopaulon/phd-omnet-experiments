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

network DADCA_ACK_n50_u16_a8000m2_5
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
        @display("i=misc/sensor;p=223.27,7838.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=885.31,7883.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1450.96,7966.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1823.56,7885.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4225.89,7839.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4726.33,7964.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5061.35,7754.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1674.54,7543.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2215.57,7589.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3150.29,7726.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3584.49,7681.27");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5642.35,7685.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=35.28,7282.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=727.08,7461.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1078.33,7471.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2549.66,7390.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3179.08,7356.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4191.87,7381.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4625.44,7486.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5987.39,7395.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=461.43,7172.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1769.99,7130.73");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2119.82,7223.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2839.15,7187.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3513.65,7189.19");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5035.26,7168.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5635.16,7113.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=101.90,6768.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2424.52,6929.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3201.50,6966.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3861.92,6806.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4364.17,6792.82");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5454.41,6779.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5970.08,7005.67");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=966.92,6754.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1418.74,6677.27");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1977.33,6749.08");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2794.15,6674.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3471.12,6613.41");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5001.26,6599.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5749.21,6588.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=513.38,6379.06");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2383.68,6402.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2983.87,6367.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=113.22,6171.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1159.94,6183.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1571.39,6248.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1926.14,6216.97");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3527.44,6263.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4088.23,6200.86");
    }

}

