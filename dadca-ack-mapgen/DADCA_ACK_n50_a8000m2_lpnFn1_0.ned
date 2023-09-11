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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=30.01,152.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=58.06,211.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=115.28,177.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=228.12,74.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=189.25,126.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=331.30,32.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=385.87,69.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=420.27,148.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=505.37,183.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=494.91,266.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=566.78,219.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=648.60,198.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=756.62,149.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=718.40,79.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=866.46,120.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=939.10,164.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=806.29,191.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=813.92,291.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=904.73,267.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=870.41,353.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=878.24,213.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=801.65,87.67");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=684.78,137.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=667.24,0.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=592.20,55.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=577.20,122.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=413.91,249.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=326.15,277.15");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=263.65,163.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=264.16,248.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=199.77,287.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=496.86,105.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=459.57,14.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=541.38,4.09");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=753.91,18.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1000.40,107.77");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=974.65,32.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1063.53,226.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1052.96,145.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=908.73,66.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=340.11,139.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=247.79,2.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=675.51,281.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=609.37,264.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1039.08,34.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=842.21,33.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=704.58,221.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=743.89,306.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=695.94,386.91");
    }

}

