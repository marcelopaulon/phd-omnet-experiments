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

network DADCA_ACK_n50_u16_a8000m2_2
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
        @display("i=misc/sensor;p=1466.99,7989.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2348.14,7866.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2710.45,7760.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3234.08,7970.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4365.47,7793.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5735.81,7764.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=979.17,7687.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1599.95,7640.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3810.22,7688.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4759.93,7718.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5357.39,7647.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=399.38,7318.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1156.70,7313.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2340.66,7438.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2976.16,7490.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3393.22,7301.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4810.65,7328.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5955.62,7440.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1750.38,7107.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2113.44,7021.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3014.63,7088.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3827.82,7015.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4465.51,7233.19");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5449.01,7137.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5920.04,7073.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=315.18,6824.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1040.06,6910.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2640.28,6836.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4177.76,6987.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3064.05,6731.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3545.47,6741.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4117.43,6614.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4686.30,6629.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5070.91,6723.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5411.65,6636.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5790.80,6673.02");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=168.15,6433.19");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=848.73,6501.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1429.95,6502.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1891.74,6443.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3180.71,6352.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4429.40,6351.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=73.11,6048.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=472.40,6230.28");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1115.36,6127.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1763.63,6023.12");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2638.93,6217.32");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4146.96,6088.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5166.30,6089.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5669.60,6061.66");
    }

}

