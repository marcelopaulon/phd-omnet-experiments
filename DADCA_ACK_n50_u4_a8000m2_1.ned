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

network DADCA_ACK_n50_u4_a8000m2_1
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
        @display("i=misc/sensor;p=226.00,7955.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=780.25,7952.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1200.17,7820.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2336.54,7893.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4040.81,7969.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2247.07,7544.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2766.81,7559.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3288.58,7555.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4201.55,7525.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=344.38,7453.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=764.85,7501.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1620.98,7440.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3780.47,7326.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=539.85,7014.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1058.91,7079.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3279.13,7138.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4575.92,7207.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1972.74,6959.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2667.11,6983.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4310.47,6819.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5862.86,6920.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=606.18,6667.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1631.32,6742.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2217.98,6592.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2816.32,6639.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3283.96,6721.70");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3911.44,6734.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5232.30,6732.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=296.39,6271.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=966.24,6487.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2565.58,6381.28");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4810.09,6413.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5664.61,6294.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=655.29,6113.03");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1497.03,6129.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2254.25,6104.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2871.59,6046.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3359.14,6201.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3942.35,6064.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4972.44,6043.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=322.10,5791.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2082.89,5782.24");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4431.64,6014.31");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5310.47,5792.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5875.73,5808.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=987.56,5761.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1483.73,5688.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2997.09,5657.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3385.59,5657.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3729.16,5760.60");
    }

}

