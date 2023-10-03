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

network DADCA_ACK_n50_u8_a8000m2_4
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
        @display("i=misc/sensor;p=1955.98,7881.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4048.43,7806.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4650.79,7999.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5155.63,7765.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1551.63,7651.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2612.57,7668.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3213.94,7704.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4302.10,7564.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4668.83,7628.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5714.66,7735.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1836.24,7261.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2925.84,7472.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3818.80,7371.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4830.07,7311.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5652.67,7378.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=104.39,7157.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=476.27,7222.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1270.04,7042.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2326.83,7096.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3450.25,7219.64");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4196.74,7208.86");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5075.08,7018.36");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=383.89,6863.04");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=797.74,6789.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1816.38,6894.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2816.36,6835.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3709.78,6768.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4291.17,6868.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=47.76,6693.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1293.65,6548.33");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2076.71,6558.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4640.74,6736.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5028.32,6611.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5468.36,6713.97");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=763.87,6410.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2467.46,6412.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3016.22,6430.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5294.00,6309.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=65.06,6230.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=448.25,6196.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1820.49,6217.95");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3220.07,6020.68");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3599.17,6169.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4055.65,6218.59");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4661.24,6259.41");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=492.02,5844.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=972.10,5962.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1620.17,5778.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2609.21,5869.90");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4450.12,5966.91");
    }

}

