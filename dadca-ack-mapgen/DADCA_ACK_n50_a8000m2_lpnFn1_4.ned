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

network DADCA_ACK_n50_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=60.96,27.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=2.17,84.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=23.82,166.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=41.50,238.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=53.76,312.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=66.45,380.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=121.18,298.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=155.87,353.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=154.85,448.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=119.55,514.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=175.17,628.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=211.80,472.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=259.13,558.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=316.69,495.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=314.29,584.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=383.23,554.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=394.06,490.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=449.63,513.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=508.22,585.10");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=429.81,643.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=459.73,363.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=467.06,293.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=387.95,342.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=419.28,423.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=499.96,703.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=317.56,683.47");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=571.42,603.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=576.83,673.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=630.33,741.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=667.18,641.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=733.88,677.89");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=802.88,631.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=721.03,564.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=818.08,572.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=886.08,639.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=942.56,615.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=973.03,531.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=955.74,462.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1033.94,446.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1042.44,524.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=976.71,386.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=913.66,383.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=851.58,390.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=884.63,487.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1126.17,494.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1095.66,442.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1207.74,560.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1138.88,603.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1037.96,636.46");
    }

}

