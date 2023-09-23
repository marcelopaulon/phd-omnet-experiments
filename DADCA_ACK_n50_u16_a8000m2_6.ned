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
        @display("i=misc/sensor;p=460.24,7966.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1972.85,7831.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2397.62,7784.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3203.56,7943.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4131.91,7981.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4466.69,7800.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=922.00,7518.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1553.13,7592.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2986.01,7608.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3503.77,7618.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4071.07,7511.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5009.87,7565.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5566.49,7622.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4653.00,7327.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5302.41,7363.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=739.12,7161.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1982.80,7056.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2596.65,7236.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3057.33,7070.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3389.85,7185.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3997.25,7110.42");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5946.31,7199.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=370.25,6941.72");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=943.91,6827.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1271.56,7002.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2318.29,6766.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4348.51,6951.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=99.44,6597.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1595.48,6754.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3309.92,6758.48");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3648.08,6658.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4897.00,6693.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5410.37,6760.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=351.38,6338.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=966.85,6454.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1964.77,6423.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2340.68,6298.01");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2933.55,6486.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3312.56,6382.77");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3659.38,6275.79");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4253.02,6475.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5228.06,6285.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5630.38,6440.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2038.48,6073.56");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4029.92,6081.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4807.79,6095.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5476.49,6031.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=710.84,5988.50");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1419.13,6001.96");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2896.14,5814.20");
    }

}

