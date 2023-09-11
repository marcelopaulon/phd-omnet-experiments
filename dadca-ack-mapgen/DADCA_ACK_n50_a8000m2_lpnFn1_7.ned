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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=79.37,195.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=46.42,137.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=179.74,125.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=225.75,206.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=240.46,26.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=289.93,92.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=371.61,37.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=439.30,81.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=484.34,8.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=538.92,33.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=482.64,130.72");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=457.92,213.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=376.11,174.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=360.31,243.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=406.41,295.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=343.80,318.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=399.30,423.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=319.98,451.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=255.37,462.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=202.81,368.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=274.02,369.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=258.40,262.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=389.39,499.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=411.89,557.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=443.07,375.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=301.34,569.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=213.20,571.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=192.14,511.94");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=124.06,450.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=101.06,521.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=175.28,296.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=98.10,257.01");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=167.50,188.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=9.29,255.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=139.99,345.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=47.50,334.56");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=35.43,422.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=16.11,500.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=15.51,571.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=80.40,645.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=81.02,740.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=6.63,714.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=3.70,635.30");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=125.89,604.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=94.00,395.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=244.29,641.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=259.79,711.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=154.67,701.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=18.62,72.65");
    }

}

