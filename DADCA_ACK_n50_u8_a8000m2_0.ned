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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=54.85,7938.53");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=542.35,7900.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1013.92,7819.33");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3455.31,7886.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5055.97,7970.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5533.18,7978.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=608.25,7543.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3131.72,7720.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3969.06,7580.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4534.96,7646.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=224.71,7435.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1210.09,7310.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1805.36,7452.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2254.43,7471.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3475.12,7409.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4362.76,7332.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5056.20,7333.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5716.52,7399.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=191.61,7071.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=667.72,7152.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2582.48,7138.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3110.38,7153.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=996.02,6907.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1592.76,6891.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2090.59,6919.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3423.42,6975.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3893.61,6929.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4527.18,6971.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5262.55,7004.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5696.03,6926.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=609.52,6678.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3514.75,6633.20");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4300.78,6639.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4870.11,6536.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5285.16,6593.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=114.96,6348.97");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1130.40,6510.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1829.56,6318.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2319.71,6495.06");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3957.39,6511.89");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=559.14,6038.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=880.20,6257.73");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1459.51,6125.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2363.16,6122.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3214.31,6198.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3689.06,6182.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4251.03,6250.84");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4978.84,6145.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5454.58,6079.43");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5812.21,6246.72");
    }

}

