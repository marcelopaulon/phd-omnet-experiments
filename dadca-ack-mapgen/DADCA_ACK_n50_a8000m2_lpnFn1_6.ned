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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=181.84,94.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=86.32,13.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=181.46,8.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=140.68,170.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=134.82,236.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=73.49,170.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=40.32,263.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=10.39,176.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=205.64,216.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=225.52,303.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=270.03,352.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=314.57,302.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=273.49,220.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=19.75,115.76");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=90.57,308.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=166.31,316.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=220.79,403.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=332.58,363.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=419.93,266.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=358.03,235.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=327.53,131.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=408.43,184.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=458.95,141.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=550.16,127.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=633.40,75.36");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=642.99,142.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=660.40,220.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=604.98,295.22");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=540.41,343.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=540.90,260.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=564.82,191.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=585.27,416.86");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=739.59,247.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=734.52,312.20");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=688.82,373.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=692.93,443.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=465.85,422.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=401.68,358.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=475.52,243.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=527.19,69.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=487.97,0.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=418.26,4.15");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=382.27,82.71");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=447.72,82.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=635.37,499.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=692.76,521.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=632.54,348.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=795.18,425.26");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=843.72,380.21");
    }

}

