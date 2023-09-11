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
        @display("i=misc/sensor,p=71.38,13.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=144.21,24.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=189.42,88.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=11.45,91.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=228.16,18.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=266.35,151.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=272.00,214.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=351.86,262.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=335.04,184.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=197.62,277.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=278.23,291.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=238.21,340.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=325.48,351.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=171.26,359.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=108.61,377.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=124.87,304.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=14.70,312.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=63.79,257.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=134.30,237.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=149.94,444.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=151.69,544.12");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=58.04,553.90");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=114.61,614.19");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=31.19,660.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=45.08,729.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=216.63,536.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=267.37,471.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=16.32,381.74");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=77.87,489.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=12.34,462.43");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=66.19,426.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=407.88,226.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=445.94,161.76");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=554.43,249.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=535.56,324.96");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=604.12,319.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=659.13,381.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=706.91,437.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=469.38,324.89");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=583.51,388.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=790.04,449.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=790.70,378.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=843.51,347.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=842.56,500.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=868.13,427.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=763.15,564.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=783.56,648.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=834.77,594.25");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=893.21,540.32");
    }

}

