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

network DADCA_ACK_n50_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=29.59,154.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=47.87,57.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=112.71,2.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=103.01,178.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=192.24,27.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=203.88,154.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=101.68,267.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=199.72,268.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=271.76,149.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=264.67,81.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=74.82,322.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=58.18,393.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=0.16,418.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=142.34,408.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=200.05,437.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=129.35,506.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=55.83,508.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=116.23,567.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=237.32,492.32");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=226.68,374.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=292.92,305.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=369.05,253.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=308.12,242.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=348.44,192.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=447.56,180.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=428.91,87.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=484.07,17.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=506.74,85.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=534.67,170.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=593.31,136.46");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=565.26,229.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=637.72,257.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=702.77,241.02");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=725.65,306.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=751.23,149.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=805.81,119.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=657.41,157.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=597.56,61.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=671.06,82.68");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=489.83,248.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=472.86,329.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=405.76,328.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=355.79,381.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=31.94,219.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=9.45,311.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=175.30,330.58");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=429.72,241.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=389.26,443.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=609.62,362.21");
    }

}

