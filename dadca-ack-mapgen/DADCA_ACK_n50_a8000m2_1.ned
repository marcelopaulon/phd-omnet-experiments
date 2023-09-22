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

network DADCA_ACK_n50_a8000m2_1
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
        @display("i=misc/sensor;p=230.02,7994.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=302.02,7982.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=408.81,7959.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=460.34,7996.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=523.85,7999.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=596.68,7958.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=673.81,7968.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=830.32,7986.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=207.25,7915.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=268.62,7924.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=473.02,7923.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=786.95,7938.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=849.30,7926.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=952.32,7940.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=377.80,7898.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=541.38,7913.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=731.77,7878.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=996.79,7870.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=125.98,7856.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=201.31,7840.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=458.10,7840.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=605.99,7831.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=665.22,7858.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=817.82,7844.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=920.87,7845.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=299.78,7811.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=381.84,7825.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=525.51,7825.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=733.68,7800.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=866.23,7800.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=970.85,7784.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=75.20,7780.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=168.65,7751.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=355.61,7761.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=529.04,7763.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=614.04,7767.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=790.38,7777.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=922.22,7745.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=272.99,7735.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=444.85,7720.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=675.35,7730.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=792.28,7712.35");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=27.81,7665.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=119.67,7678.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=197.06,7673.51");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=318.33,7657.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=491.96,7680.94");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=592.49,7680.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=853.68,7667.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=971.10,7693.97");
    }

}

