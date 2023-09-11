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
        @display("i=misc/sensor,p=175.69,113.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=197.65,30.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=279.65,31.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=262.91,90.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=359.45,59.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=428.54,18.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=505.56,69.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=421.20,155.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=495.31,184.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=425.32,253.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=366.12,295.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=321.48,357.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=259.35,403.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=214.64,466.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=227.90,535.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=144.05,483.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=129.84,565.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=212.18,594.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=297.67,519.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=351.77,432.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=174.47,363.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=149.09,273.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=221.75,206.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=116.20,212.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=344.92,174.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=314.40,244.26");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=91.30,319.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=2.54,296.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=79.35,414.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=7.72,403.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=18.67,230.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=56.87,507.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=41.53,584.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=96.86,621.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=371.99,504.90");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=437.16,503.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=427.57,574.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=358.24,599.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=498.62,619.59");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=513.08,707.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=591.05,744.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=598.46,813.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=645.26,773.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=531.67,788.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=488.16,908.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=482.09,976.02");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=534.58,1010.85");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=602.68,967.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=628.48,1026.98");
    }

}

