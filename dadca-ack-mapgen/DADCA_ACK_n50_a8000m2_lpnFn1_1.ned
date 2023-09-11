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
        @display("i=misc/sensor,p=28.03,121.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=32.10,56.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=93.19,24.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=238.08,38.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=166.28,26.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=210.75,174.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=309.85,180.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=372.05,156.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=328.42,94.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=289.56,245.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=191.96,95.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=136.07,189.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=168.72,240.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=102.05,254.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=38.56,293.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=22.53,212.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=249.30,327.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=401.06,303.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=399.72,386.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=365.37,455.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=458.73,101.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=411.41,22.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=518.93,30.08");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=451.65,206.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=268.30,121.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=349.74,339.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=347.70,263.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=535.70,222.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=602.02,148.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=523.11,158.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=489.80,267.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=481.63,367.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=633.55,343.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=654.65,421.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=575.01,370.10");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=542.76,295.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=552.76,440.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=602.36,487.94");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=473.44,431.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=620.97,252.82");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=710.48,245.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=699.29,180.62");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=632.04,62.74");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=710.56,82.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=677.45,1.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=784.15,169.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=602.15,7.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=328.05,8.23");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=523.85,90.62");
    }

}

