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

network DADCA_ACK_n50_u3_a8000m2_6
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
        @display("i=misc/sensor;p=680.03,7885.08");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1791.54,7761.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2167.91,7949.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2668.22,7999.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3737.22,7855.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4099.89,7990.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4946.83,7837.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5967.38,7978.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=155.74,7518.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3293.77,7514.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4339.98,7685.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5523.13,7550.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=531.67,7452.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1140.20,7391.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1884.06,7325.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2274.04,7281.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2768.67,7486.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4031.68,7372.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4980.37,7421.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=772.64,7158.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3089.91,7010.38");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3619.96,7036.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4412.21,7198.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5596.88,7056.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5994.50,7046.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=149.08,6990.49");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=519.97,6910.94");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1132.11,6848.47");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1757.21,6876.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2157.19,6813.76");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4830.06,6946.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5149.79,6770.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2733.44,6643.04");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3762.77,6517.27");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4128.70,6725.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4523.87,6688.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=213.62,6384.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=682.39,6355.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1685.13,6472.32");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2275.02,6379.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2924.18,6343.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3403.99,6498.06");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5120.93,6338.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5484.25,6457.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5900.00,6311.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1025.69,6027.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1499.51,6167.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2708.03,6064.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3196.69,6107.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3760.89,6153.95");
    }

}

