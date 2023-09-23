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

network DADCA_ACK_n50_u16_a8000m2_7
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
        @display("i=misc/sensor;p=1081.42,7811.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1521.07,7835.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2419.53,7772.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2804.69,7797.77");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3354.14,7999.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4163.66,7908.13");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5173.30,7903.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5624.70,7841.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5990.26,7944.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=178.88,7718.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=538.90,7580.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1970.46,7680.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3704.88,7708.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4074.35,7512.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4645.51,7672.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=939.64,7468.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1236.52,7265.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1794.64,7369.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2404.36,7383.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3179.10,7422.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5289.93,7493.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=227.60,7102.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=702.68,7169.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1939.07,7050.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2842.91,7166.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3179.65,7024.61");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3522.96,7177.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3889.06,7195.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4458.19,7189.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4901.61,7030.59");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5644.68,7142.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=836.40,6819.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=530.13,6536.43");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1134.39,6632.02");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1554.71,6703.88");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2106.10,6584.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2678.03,6624.91");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3627.83,6586.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4375.80,6703.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4734.80,6559.86");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5234.31,6603.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5847.33,6721.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3025.05,6440.32");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=991.01,6070.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1745.05,6209.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2194.19,6187.12");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3563.80,6219.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4498.38,6030.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4878.05,6102.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5604.32,6220.53");
    }

}

