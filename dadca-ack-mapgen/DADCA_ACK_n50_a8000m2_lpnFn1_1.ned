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

network DADCA_ACK_n50_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=72.15,36.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=37.49,88.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=22.47,164.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=101.05,182.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=177.39,164.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=243.18,148.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=309.94,136.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=174.34,83.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=164.30,12.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=171.26,289.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=182.01,363.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=193.82,447.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=139.96,420.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=88.04,389.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=23.27,403.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=47.02,498.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=89.85,568.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=149.16,551.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=8.90,648.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=63.08,779.29");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=84.37,682.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=18.23,882.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=138.53,860.82");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=156.73,949.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=211.66,870.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=87.91,954.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=146.26,768.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=282.61,998.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=246.87,1066.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=262.14,929.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=341.15,928.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=372.00,853.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=420.98,926.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=377.37,1016.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=526.95,841.24");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=473.50,804.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=456.75,734.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=519.17,701.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=525.62,772.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=587.06,784.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=611.88,845.05");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=623.34,709.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=481.55,591.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=469.01,657.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=568.52,648.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=664.81,768.50");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=754.34,774.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=684.22,863.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=733.75,949.39");
    }

}

