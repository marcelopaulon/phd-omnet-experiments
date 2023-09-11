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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=31.62,85.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=104.01,1.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=174.31,124.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=204.84,67.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=267.30,49.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=320.90,77.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=287.53,143.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=394.12,82.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=419.17,14.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=487.52,80.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=450.52,131.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=370.26,198.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=295.71,219.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=226.52,186.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=460.63,235.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=491.51,293.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=393.74,288.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=530.65,22.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=599.43,26.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=682.91,59.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=641.20,112.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=590.69,193.82");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=644.55,227.57");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=705.44,267.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=745.13,220.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=702.80,175.71");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=861.85,199.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=806.23,131.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=574.46,321.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=524.89,355.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=494.29,431.91");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=625.57,367.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=659.35,432.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=625.43,287.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=813.39,65.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=730.00,110.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=738.81,15.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=876.80,55.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=919.92,125.57");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=983.34,196.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1062.43,213.12");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1071.77,131.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1045.24,42.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1123.43,10.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1166.61,56.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1003.99,126.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=988.31,13.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1134.20,178.18");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1194.56,158.01");
    }

}

