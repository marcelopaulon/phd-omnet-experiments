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
        @display("i=misc/sensor,p=197.75,88.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=287.74,143.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=335.21,64.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=410.24,122.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=426.23,191.51");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=138.17,4.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=147.43,153.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=46.99,158.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=184.85,209.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=107.60,245.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=29.49,226.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=257.52,21.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=476.13,99.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=490.63,11.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=428.47,46.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=568.51,37.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=353.12,193.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=372.33,257.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=267.84,260.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=491.81,244.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=431.56,286.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=457.66,371.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=513.04,397.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=537.41,486.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=470.08,552.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=434.32,500.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=411.43,422.49");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=370.74,468.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=301.55,436.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=319.69,525.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=330.06,596.07");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=199.06,341.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=230.27,413.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=187.18,517.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=172.56,597.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=105.30,489.14");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=76.04,425.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=118.59,365.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=273.56,341.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=337.14,313.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=175.21,276.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=175.98,453.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=33.81,471.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=374.42,649.52");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=460.09,690.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=544.92,664.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=487.59,632.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=450.66,794.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=538.21,804.54");
    }

}

