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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=181.37,122.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=259.56,62.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=88.18,204.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=185.94,184.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=57.33,280.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=124.74,279.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=298.49,121.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=347.15,56.62");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=489.10,84.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=582.54,70.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=566.95,163.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=639.78,145.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=518.97,25.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=436.33,1.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=637.54,275.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=612.66,350.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=604.48,437.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=547.04,411.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=701.13,469.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=742.23,422.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=838.49,397.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=724.29,309.87");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=775.57,342.08");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=848.31,307.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=840.72,239.30");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=929.62,208.26");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=779.95,229.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=746.73,162.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=728.85,68.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=676.68,27.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=839.66,141.91");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=786.47,102.46");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=995.50,163.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=982.47,83.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=896.40,93.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=849.65,44.51");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=662.07,214.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=587.58,239.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1045.60,39.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=939.68,326.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=995.18,251.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1126.79,203.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1205.50,195.56");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1194.50,281.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1167.34,337.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1094.83,310.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=480.65,341.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=420.80,412.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=353.71,354.09");
    }

}

