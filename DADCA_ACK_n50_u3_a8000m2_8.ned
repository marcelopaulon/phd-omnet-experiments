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

network DADCA_ACK_n50_u3_a8000m2_8
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
        @display("i=misc/sensor;p=1951.29,7885.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2534.74,7925.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3110.68,7891.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3989.67,7794.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5154.52,7810.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5611.97,7794.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=37.40,7699.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=838.41,7579.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1238.13,7698.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1866.13,7508.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3415.36,7696.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4474.35,7650.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1375.87,7335.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2436.97,7481.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3118.87,7503.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4343.80,7303.32");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5684.01,7278.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=89.17,7063.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=459.38,7034.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=810.06,7217.13");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1748.37,7117.38");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2802.14,7127.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3659.49,7069.25");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4968.53,7197.56");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=941.20,6814.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1420.58,6935.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2521.18,6904.35");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4583.54,6861.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5847.27,6908.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=154.25,6645.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=610.67,6612.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1286.03,6583.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3085.64,6725.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3472.77,6566.23");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3845.96,6597.56");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4227.18,6752.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4838.34,6580.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5275.25,6696.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=966.44,6440.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1822.05,6441.82");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2278.30,6458.63");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2785.14,6317.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5452.66,6334.76");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=625.67,6129.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2351.72,6048.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3106.32,6042.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3795.55,6190.39");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4415.81,6256.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4978.31,6229.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5900.92,6241.10");
    }

}

