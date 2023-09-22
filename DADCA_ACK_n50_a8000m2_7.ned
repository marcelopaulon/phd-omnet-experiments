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

network DADCA_ACK_n50_a8000m2_7
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
        @display("i=misc/sensor;p=3890.75,7823.52");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4736.82,7955.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5355.42,7895.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1316.53,7587.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1930.37,7738.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5161.60,7580.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5634.70,7649.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2415.10,7284.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4202.91,7377.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4577.53,7484.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1539.42,7167.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2732.03,7071.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4891.48,7054.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5475.78,7058.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1256.26,6919.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2010.76,6821.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3972.73,6944.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4429.25,6898.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=109.86,6629.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=903.06,6741.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2610.07,6604.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2967.39,6697.26");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3428.32,6695.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4959.46,6672.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=427.53,6413.57");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=783.60,6355.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1186.16,6412.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1590.20,6422.12");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2387.93,6284.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3076.35,6308.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3521.50,6273.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4141.87,6346.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5138.46,6274.64");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5494.36,6365.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1.81,6188.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1954.08,6181.65");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4477.77,6213.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5764.47,6033.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=606.43,5846.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1233.65,5895.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2280.37,5841.82");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3032.57,5944.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4323.49,5818.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4675.44,5823.59");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5176.00,5925.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=86.90,5755.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1110.15,5532.16");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3415.71,5745.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3783.86,5680.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=735.90,5518.94");
    }

}

